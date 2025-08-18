#!/usr/bin/env python3
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from protocol.srv import CameraService
import threading
import subprocess

from motion_controller import MotionController
from straight_detector import detect_deviation
from angle_turn import LineDistanceDetector, compute_line_offset
from qrcode_text_detector import text_detector


def get_namespace():
    """自动检测 ROS2 命名空间"""
    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            check=True,
        )
        for line in result.stdout.splitlines():
            if "mi_" in line:
                parts = line.strip().split("/")
                if len(parts) > 1:
                    return "/" + parts[1]
    except subprocess.CalledProcessError as e:
        print("获取命名空间失败:", e)
    return "/"  # 默认根命名空间


# ---------------- PID 控制器 ----------------
class PID:
    def __init__(
        self, kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0), integral_limit=None
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limit = integral_limit
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.perf_counter()

    def reset(self):
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.perf_counter()

    def calculate(self, setpoint, measurement):
        now = time.perf_counter()
        dt = now - self.last_time
        if dt <= 0.0:
            dt = 1e-6
        error = setpoint - measurement
        self.error_sum += error * dt
        if self.integral_limit is not None:
            self.error_sum = max(
                min(self.error_sum, self.integral_limit), -self.integral_limit
            )
        error_delta = (error - self.last_error) / dt
        output = (
            (self.kp * error) + (self.ki * self.error_sum) + (self.kd * error_delta)
        )
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        self.last_error = error
        self.last_time = now
        return output


# ---------------- 主控制节点 ----------------
class PIDController(Node):
    def __init__(self):
        super().__init__("cyberdog_controller", namespace=get_namespace())

        # --- 原有初始化保持不变 ---
        self.pid = PID(
            kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0), integral_limit=5.0
        )
        self.pid_used = False
        self.motioncontroller = MotionController()
        self.motioncontroller.control_run()

        # 底线距离检测
        self.distance_detector = LineDistanceDetector(roi_width=20, smooth_window=5)
        self.distance_detector_used = False

        self.active_timers = {}
        self.bridge = CvBridge()

        # 相机初始化
        self.ai_camera_get_ready = False
        self.rgb_camera_get_ready = False
        self.get_ready = False

        # 阶段布尔值
        self.first_straight = False
        self.first_turn = False
        self.second_straight = False
        self.text1_get = False
        self.text1_result = None

        # --- 相机服务客户端（先用占位名创建，后续会解析真实服务名） ---
        self.client = self.create_client(CameraService, "camera_service")
        self.get_logger().info(
            f"相机服务客户端已创建（占位名）: {self.client.srv_name}"
        )

        # OpenCV 窗口
        cv2.namedWindow("CyberDog Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CyberDog Camera", 640, 480)
        cv2.namedWindow("RGB Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RGB Camera Feed", 800, 600)

        # 延迟启动：spin 起来后再做服务解析和相机控制
        self._camera_boot_timer = self.create_timer(1.0, self.try_boot_camera_once)

    # ---------- 服务名自动发现 ----------
    def _resolve_camera_service_name(self) -> str:
        """
        在当前可见的服务里查找以 'camera_service' 结尾的完整名。
        优先匹配当前命名空间；找不到再退而求其次用任意匹配。
        """
        services = self.get_service_names_and_types()
        ns = self.get_namespace().rstrip("/")
        candidates_in_ns = []
        candidates_all = []

        for name, _types in services:
            if name.endswith("/camera_service") or name == "/camera_service":
                candidates_all.append(name)
                # 在当前命名空间下的服务（/ns/...）
                if (ns and name.startswith(f"/{ns}/")) or (
                    not ns and name == "/camera_service"
                ):
                    candidates_in_ns.append(name)

        if candidates_in_ns:
            return candidates_in_ns[0]
        if candidates_all:
            return candidates_all[0]
        return ""  # 没找到

    def try_boot_camera_once(self):
        # 只跑一次
        if hasattr(self, "_camera_boot_timer") and self._camera_boot_timer is not None:
            self._camera_boot_timer.cancel()
            self._camera_boot_timer = None

        # 解析真实服务名
        real_srv = self._resolve_camera_service_name()
        if not real_srv:
            self.get_logger().warn("未发现 camera_service（2 秒后重试）")
            # 2 秒后再试一次
            self._camera_boot_timer = self.create_timer(2.0, self.try_boot_camera_once)
            return

        if real_srv != self.client.srv_name:
            self.get_logger().info(f"发现真实服务名: {real_srv} ，重新绑定客户端")
            # 重新创建指向真实服务名的客户端
            self.client = self.create_client(CameraService, real_srv)

        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"服务 {self.client.srv_name} 未就绪（2 秒后重试）")
            self._camera_boot_timer = self.create_timer(2.0, self.try_boot_camera_once)
            return

        self.get_logger().info("相机服务已就绪，开始控制相机")
        self.control_camera()

    # ---------- 相机控制：先 STOP，超时则直接 START ----------
    def control_camera(self):
        if not self.client.service_is_ready():
            self.get_logger().warn("服务还没就绪，稍后重试 control_camera")
            self.create_timer(1.0, self.control_camera)
            return

        stop_request = CameraService.Request()
        stop_request.command = CameraService.Request.STOP_IMAGE_PUBLISH
        stop_request.height = 0
        stop_request.width = 0
        stop_request.fps = 0

        self.get_logger().info(f"发送停止相机请求到 {self.client.srv_name} ...")
        stop_future = self.client.call_async(stop_request)

        # 2 秒 watchdog：超时直接尝试启动
        def stop_watchdog():
            if not stop_future.done():
                self.get_logger().warn("STOP 请求超时（2s），跳过 STOP，直接尝试 START")
                self.start_camera()

        self.create_timer(2.0, stop_watchdog)

        # 真正回调
        stop_future.add_done_callback(self.stop_camera_callback)

    def stop_camera_callback(self, future):
        try:
            self.get_logger().info("stop_camera_callback 触发")
            response = future.result()
            if response.result == CameraService.Response.RESULT_SUCCESS:
                self.get_logger().info("相机已停止（服务端返回 SUCCESS）")
            else:
                self.get_logger().warn(
                    f"停止相机失败（服务端返回 {response.result}），继续尝试启动"
                )
        except Exception as e:
            self.get_logger().error(f"停止相机服务调用失败: {e}")
        finally:
            self.start_camera()

    def start_camera(self):
        if not self.client.service_is_ready():
            self.get_logger().warn("服务未就绪，延迟 1 秒再 START")
            self.create_timer(1.0, self.start_camera)
            return

        start_request = CameraService.Request()
        start_request.command = CameraService.Request.START_IMAGE_PUBLISH
        start_request.height = 480
        start_request.width = 640
        start_request.fps = 30

        self.get_logger().info(f"发送启动相机请求到 {self.client.srv_name} ...")
        start_future = self.client.call_async(start_request)

        # 2 秒 watchdog：如果 START 也超时，打印提示并不再阻塞
        def start_watchdog():
            if not start_future.done():
                self.get_logger().warn(
                    "START 请求超时（2s），可能服务端未实现响应；将继续订阅话题。"
                )
                # 即使 start 超时，也去订阅（有些服务端默认就在发布）
                self.subscribe_cameras()

        self.create_timer(2.0, start_watchdog)

        start_future.add_done_callback(self.start_camera_callback)

    def start_camera_callback(self, future):
        try:
            self.get_logger().info("start_camera_callback 触发")
            response = future.result()
            if response.result == CameraService.Response.RESULT_SUCCESS:
                self.get_logger().info("相机已成功启动")
            else:
                self.get_logger().warn(
                    f"启动相机失败（服务端返回 {response.result}），仍尝试订阅话题"
                )
        except Exception as e:
            self.get_logger().error(f"启动相机服务调用失败: {e}")
        finally:
            self.subscribe_cameras()

    # ----------- 统一订阅两个相机 -----------
    def subscribe_cameras(self):
        # 定义一个可靠传输的 QoSProfile
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # CyberDog 相机
        cyberdog_topic = f"{self.get_namespace()}/image"  # 建议用绝对路径
        self.create_subscription(
            Image, cyberdog_topic, self.image_callback, qos_reliable
        )
        self.get_logger().info(f"已订阅 CyberDog 相机话题: {cyberdog_topic}")

        # RGB 相机
        rgb_topic = "/image_rgb"
        self.create_subscription(Image, rgb_topic, self.rgb_callback, qos_reliable)
        self.get_logger().info(f"已订阅 RGB 相机话题: {rgb_topic}")

    # ----------- CyberDog 相机回调 -----------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("CyberDog Camera", cv_image)
            key = cv2.waitKey(1)

            if not self.ai_camera_get_ready:
                self.get_logger().info("RGB相机初始化完成！")
                self.ai_camera_get_ready = True

            elif self.second_straight and not self.text1_result:
                self.get_logger().info("机器狗图像识别中")
                if self.text1_get:
                    self.motioncontroller.cmd_msg.motion_id = 308
                    self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
                    self.motioncontroller.cmd_msg.vel_des = [
                        (random.random() - 0.5) * 0.3,
                        0.0,
                        0.0,
                    ]
                    self.start_flag_timer("text1_get", 2.0, False)
                else:
                    self.motioncontroller.cmd_msg.motion_id = 111
                    self.text1_result = text_detector(cv_image)
                    self.text1_get = True

        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    # ----------- RGB 相机回调 -----------
    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("RGB Camera Feed", cv_image)
            cv2.waitKey(1)

            # 控制逻辑示例
            if not self.rgb_camera_get_ready:
                self.get_logger().info("AI相机初始化完成！")
                self.rgb_camera_get_ready = True

            elif not self.get_ready:
                self.get_logger().info("机器狗整体初始化中")
                self.start_flag_timer("get_ready", 10.0, True)

            # 第一次直线行驶
            elif not self.first_straight:
                self.get_logger().info("机器狗第一次直行")
                self.motioncontroller.cmd_msg.motion_id = 308
                self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
                self.start_flag_timer("first_straight", 4.5)
                self.start_flag_timer("pid_used", 4.5, False)

                # PID 控制初始化
                if not self.pid_used:
                    self.pid = PID(kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0))
                    self.pid_used = True

                deviation = detect_deviation(cv_image)
                print("偏航角:", deviation)

                base_speed = 0.2
                # 用 PID 计算修正量
                correction = 1.5 * self.pid.calculate(0.0, deviation)
                print("比例系数:", correction)

                self.motioncontroller.cmd_msg.vel_des = [
                    base_speed,
                    0.0,
                    base_speed * correction,
                ]

            # 第一次直角转弯
            elif not self.first_turn:
                self.get_logger().info("机器狗第一次转弯")
                self.motioncontroller.cmd_msg.motion_id = 308
                self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
                self.motioncontroller.cmd_msg.vel_des = [0.0, 0.0, -0.58]
                self.start_flag_timer("first_turn", 3.0)

            # 第二次直线行驶
            elif not self.second_straight:
                self.get_logger().info("机器狗第二次直行")
                self.motioncontroller.cmd_msg.motion_id = 308
                self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]

                # 底线距离计算器初始化
                if not self.distance_detector_used:
                    self.distance_detector = LineDistanceDetector(
                        roi_width=20, smooth_window=5
                    )
                    self.distance_detector_used = True

                # PID 控制初始化
                if not self.pid_used:
                    self.pid = PID(kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0))
                    self.pid_used = True

                deviation = detect_deviation(cv_image)
                print("偏航角:", deviation)

                base_speed = 0.2
                # 用 PID 计算修正量
                correction = 2.0 * self.pid.calculate(0.0, deviation)
                print("比例系数:", correction)

                self.motioncontroller.cmd_msg.vel_des = [
                    base_speed,
                    0.0,
                    base_speed * correction,
                ]

                # 距离检查
                distance = self.distance_detector.detect_line_distance(cv_image)
                print("当前距离为", distance)
                if distance < 400:
                    print("second_straight已经置为True")
                    self.distance_detector_used = False
                    self.pid_used = False
                    self.second_straight = True

            elif self.text1_result:
                print("图像识别结果为:", self.text1_result)
                self.motioncontroller.cmd_msg.motion_id = 101
                time.sleep(0.5)

            # # 直线行驶
            # elif not self.first_straight:
            #     self.get_logger().info("机器狗第一次直行")
            #     self.motioncontroller.cmd_msg.motion_id = 308
            #     self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
            #     self.start_flag_timer("first_straight", 12.0)

            #     # 底线距离计算器初始化
            #     if not self.distance_detector_used:
            #         self.distance_detector = LineDistanceDetector(
            #             roi_width=20, smooth_window=5
            #         )
            #         self.distance_detector_used = True

            #     # PID 控制初始化
            #     if not self.pid_used:
            #         self.pid = PID(kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0))
            #         self.pid_used = True

            #     deviation = detect_deviation(cv_image)
            #     print("偏航角:", deviation)

            #     base_speed = 0.2
            #     # 用 PID 计算修正量
            #     correction = 2.0 * self.pid.calculate(0.0, deviation)
            #     print("比例系数:", correction)

            #     self.motioncontroller.cmd_msg.vel_des = [
            #         base_speed,
            #         0.0,
            #         base_speed * correction,
            #     ]

            #     # 距离检查
            #     distance = self.distance_detector.detect_line_distance(cv_image)
            #     print("当前距离为", distance)
            #     if distance < 130:
            #         self.distance_detector_used = False
            #         self.pid_used = False
            #         self.first_straight = True

            # # 角度调节
            # elif not self.adjust_line:
            #     self.get_logger().info("机器狗转弯前角度调节")
            #     self.start_flag_timer("pid_used", 5.0, False)
            #     self.start_flag_timer("adjust_line", 5.0)
            #     # PID 控制初始化
            #     if not self.pid_used:
            #         self.pid = PID(kp=0.6, ki=0.0, kd=0.4, output_limits=(-1.0, 1.0))
            #         self.pid_used = True

            #     deviation = compute_line_offset(cv_image)
            #     print("偏航角:", deviation)

            #     base_speed = 0.3
            #     # 用 PID 计算修正量
            #     correction = self.pid.calculate(0.0, deviation)
            #     print("比例系数:", correction)

            #     self.motioncontroller.cmd_msg.rpy_des = [0.0, 0.3, 0.0]
            #     self.motioncontroller.cmd_msg.vel_des = [
            #         0.0,
            #         0.0,
            #         base_speed * correction,
            #     ]

            # # 前进补正
            # elif not self.straight_fix:
            #     self.get_logger().info("机器狗前进补正")

            #     self.motioncontroller.cmd_msg.vel_des = [0.2, 0.0, 0.0]

            #     # 底线距离计算器初始化
            #     if not self.distance_detector_used:
            #         self.distance_detector = LineDistanceDetector(
            #             roi_width=20, smooth_window=5
            #         )
            #         self.distance_detector_used = True

            #     # 距离检查
            #     distance = self.distance_detector.detect_line_distance(cv_image)
            #     print("当前距离为", distance)
            #     if distance < 40:
            #         self.distance_detector_used = False
            #         self.straight_fix = True

            # # 直角转弯
            # elif not self.first_turn:
            #     self.get_logger().info("机器狗第一次转弯")
            #     self.motioncontroller.cmd_msg.motion_id = 308
            #     self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
            #     self.motioncontroller.cmd_msg.vel_des = [0.0, 0.0, 0.62]
            #     self.motioncontroller.cmd_msg.rpy_des = [0.0, 0.0, 0.0]
            #     self.start_flag_timer("first_turn", 3.0)

            # else:
            #     self.motioncontroller.cmd_msg.motion_id = 101
            #     time.sleep(0.5)

        except Exception as e:
            self.get_logger().error(f"RGB 图像处理错误: {str(e)}")

    def start_flag_timer(
        self, flag_name: str, duration_sec: float, target: bool = True
    ):
        """
        ROS2 安全的一次性计时器：
        duration_sec 秒后将 self.flag_name 翻转一次
        并销毁计时器，防止再次反转
        """
        # 如果已有计时器正在运行，先取消它
        if flag_name in self.active_timers:
            return

        def timer_callback():
            # 置 target
            setattr(self, flag_name, target)
            self.get_logger().info(f"{flag_name} 已置为 {target}")

            # 停止并清理计时器
            timer.cancel()
            self.active_timers.pop(flag_name, None)

        # 使用 ROS2 计时器（周期性的，但我们会在第一次回调里 cancel）
        timer = self.create_timer(duration_sec, timer_callback)
        self.active_timers[flag_name] = timer


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断")
    finally:
        # 停止相机
        try:
            if hasattr(node, "motioncontroller"):
                node.motioncontroller.stop_thread()
            if node.client.service_is_ready():
                stop_request = CameraService.Request()
                stop_request.command = CameraService.Request.STOP_IMAGE_PUBLISH
                node.client.call_async(stop_request)
                node.get_logger().info("发送最终停止相机请求")
        except Exception as e:
            node.get_logger().error(f"停止相机失败: {e}")

        # 销毁 OpenCV 窗口
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
