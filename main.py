#!/usr/bin/env python3
import random
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from protocol.srv import CameraService
import threading
import subprocess
from typing import Optional

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

        # CallbackGroup：订阅/服务/定时分组，减少互相阻塞
        self.cbg_sub = ReentrantCallbackGroup()
        self.cbg_srv = MutuallyExclusiveCallbackGroup()
        self.cbg_misc = MutuallyExclusiveCallbackGroup()

        # 控制器与状态变量
        self.pid = PID(
            kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0), integral_limit=5.0
        )
        self.pid_used = False
        self.motioncontroller = MotionController()
        self.motioncontroller.control_run()

        self.distance_detector = LineDistanceDetector(roi_width=20, smooth_window=5)
        self.distance_detector_used = False

        self.bridge = CvBridge()

        # 相机初始化标记
        self.ai_camera_get_ready = False
        self.rgb_camera_get_ready = False
        self.get_ready = False

        # 任务阶段
        self.first_straight = False
        self.first_turn = False
        self.second_straight = False
        self.text1_get = False
        self.text1_result = None
        self.first_adjust = False
        self.first_fix = False

        # 线程安全：共享帧
        self._lock = threading.RLock()
        self._last_ai_frame: Optional[any] = None
        self._last_rgb_frame: Optional[any] = None

        # 线程级定时器
        self.active_timers = {}  # name -> threading.Timer
        self._stop_event = threading.Event()

        # 相机服务客户端（占位名，后续解析）
        self.client = self.create_client(
            CameraService, "camera_service", callback_group=self.cbg_srv
        )
        self.get_logger().info(
            f"相机服务客户端已创建（占位名）: {self.client.srv_name}"
        )

        # OpenCV 窗口（统一在状态机线程刷新显示）
        cv2.namedWindow("CyberDog Camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CyberDog Camera", 640, 480)
        cv2.namedWindow("RGB Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("RGB Camera Feed", 800, 600)

        # 延迟启动：spin 起来后再做服务解析和相机控制
        self._camera_boot_timer = self.create_timer(
            1.0, self.try_boot_camera_once, callback_group=self.cbg_misc
        )

        # 启动控制状态机线程（独立于 ROS 回调）
        self._control_thread = threading.Thread(
            target=self._control_loop, name="control_loop", daemon=True
        )
        self._control_thread.start()

        # 帧显示线程（可选，把imshow与waitKey集中处理，避免多线程争用）
        self._display_thread = threading.Thread(
            target=self._display_loop, name="display_loop", daemon=True
        )
        self._display_thread.start()

    # ---------- 服务名自动发现 ----------
    def _resolve_camera_service_name(self) -> str:
        """
        在当前可见的服务里查找以 'camera_service' 结尾的完整名。
        优先匹配当前命名空间；找不到再退而求其次用任意匹配。
        """
        services = self.get_service_names_and_types()
        ns = self.get_namespace().lstrip(
            "/"
        )  # 注意：get_namespace() 返回前含'/'，这里对比方便
        candidates_in_ns = []
        candidates_all = []

        for name, _types in services:
            if name.endswith("/camera_service") or name == "/camera_service":
                candidates_all.append(name)
                # /ns/... 形式优先
                if ns and name.startswith(f"/{ns}/"):
                    candidates_in_ns.append(name)
                if not ns and name == "/camera_service":
                    candidates_in_ns.append(name)

        if candidates_in_ns:
            return candidates_in_ns[0]
        if candidates_all:
            return candidates_all[0]
        return ""  # 没找到

    def try_boot_camera_once(self):
        # 只跑一次，若失败会自行重挂定时器
        if hasattr(self, "_camera_boot_timer") and self._camera_boot_timer is not None:
            self._camera_boot_timer.cancel()
            self._camera_boot_timer = None

        real_srv = self._resolve_camera_service_name()
        if not real_srv:
            self.get_logger().warn("未发现 camera_service（2 秒后重试）")
            self._camera_boot_timer = self.create_timer(
                2.0, self.try_boot_camera_once, callback_group=self.cbg_misc
            )
            return

        if real_srv != self.client.srv_name:
            self.get_logger().info(f"发现真实服务名: {real_srv} ，重新绑定客户端")
            self.client = self.create_client(
                CameraService, real_srv, callback_group=self.cbg_srv
            )

        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"服务 {self.client.srv_name} 未就绪（2 秒后重试）")
            self._camera_boot_timer = self.create_timer(
                2.0, self.try_boot_camera_once, callback_group=self.cbg_misc
            )
            return

        self.get_logger().info("相机服务已就绪，开始控制相机")
        self.control_camera()

    # ---------- 相机控制：先 STOP，超时则直接 START ----------
    def control_camera(self):
        if not self.client.service_is_ready():
            self.get_logger().warn("服务还没就绪，稍后重试 control_camera")
            self.create_timer(1.0, self.control_camera, callback_group=self.cbg_misc)
            return

        stop_request = CameraService.Request()
        stop_request.command = CameraService.Request.STOP_IMAGE_PUBLISH
        stop_request.height = 0
        stop_request.width = 0
        stop_request.fps = 0

        self.get_logger().info(f"发送停止相机请求到 {self.client.srv_name} ...")
        stop_future = self.client.call_async(stop_request)

        def stop_watchdog():
            if not stop_future.done():
                self.get_logger().warn("STOP 请求超时（2s），跳过 STOP，直接尝试 START")
                self.start_camera()

        self.create_timer(2.0, stop_watchdog, callback_group=self.cbg_misc)
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
            self.create_timer(1.0, self.start_camera, callback_group=self.cbg_misc)
            return

        start_request = CameraService.Request()
        start_request.command = CameraService.Request.START_IMAGE_PUBLISH
        start_request.height = 480
        start_request.width = 640
        start_request.fps = 30

        self.get_logger().info(f"发送启动相机请求到 {self.client.srv_name} ...")
        start_future = self.client.call_async(start_request)

        def start_watchdog():
            if not start_future.done():
                self.get_logger().warn(
                    "START 请求超时（2s），可能服务端未实现响应；将继续订阅话题。"
                )
                self.subscribe_cameras()

        self.create_timer(2.0, start_watchdog, callback_group=self.cbg_misc)
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

    # ----------- 统一订阅两个相机（轻回调：只取帧与标志） -----------
    def subscribe_cameras(self):
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        cyberdog_topic = f"{self.get_namespace()}/image"
        self.create_subscription(
            Image,
            cyberdog_topic,
            self.image_callback,
            qos_reliable,
            callback_group=self.cbg_sub,
        )
        self.get_logger().info(f"已订阅 AI 相机话题: {cyberdog_topic}")

        rgb_topic = "/image_rgb"
        self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            qos_reliable,
            callback_group=self.cbg_sub,
        )
        self.get_logger().info(f"已订阅 RGB 相机话题: {rgb_topic}")

    # ----------- CyberDog 相机回调（仅缓存帧与就绪标志）-----------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._last_ai_frame = cv_image
            if not self.ai_camera_get_ready:
                self.get_logger().info("AI相机初始化完成！")
                self.ai_camera_get_ready = True
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    # ----------- RGB 相机回调（仅缓存帧与就绪标志）-----------
    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self._lock:
                self._last_rgb_frame = cv_image
            if not self.rgb_camera_get_ready:
                self.get_logger().info("RGB相机初始化完成！")
                self.rgb_camera_get_ready = True
        except Exception as e:
            self.get_logger().error(f"RGB 图像处理错误: {str(e)}")

    # ----------- 显示线程：集中处理 imshow，避免多线程抢占 -----------
    def _display_loop(self):
        rate = 1.0 / 30.0
        while not self._stop_event.is_set():
            frame1 = None
            frame2 = None
            with self._lock:
                frame1 = (
                    self._last_ai_frame.copy()
                    if self._last_ai_frame is not None
                    else None
                )
                frame2 = (
                    self._last_rgb_frame.copy()
                    if self._last_rgb_frame is not None
                    else None
                )
            try:
                if frame1 is not None:
                    cv2.imshow("CyberDog Camera", frame1)
                if frame2 is not None:
                    cv2.imshow("RGB Camera Feed", frame2)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"显示线程异常: {e}")
            time.sleep(rate)

    # ----------- 控制状态机线程：固定频率运行，不依赖回调节拍 -----------
    def _control_loop(self):
        hz = 20.0
        dt = 1.0 / hz
        last_log = 0.0

        while not self._stop_event.is_set():
            t0 = time.perf_counter()

            # 双相机皆就绪后，再进入 get_ready 阶段
            if (
                self.ai_camera_get_ready
                and self.rgb_camera_get_ready
                and not self.get_ready
            ):
                print("机器狗初始化中...")
                # 仅置一次，避免反复计时
                self.start_flag_timer("get_ready", 10.0, True)

            # 读取最新帧
            with self._lock:
                rgb = (
                    self._last_rgb_frame.copy()
                    if self._last_rgb_frame is not None
                    else None
                )
                ai = (
                    self._last_ai_frame.copy()
                    if self._last_ai_frame is not None
                    else None
                )

            # 主逻辑（对应你原先 rgb_callback 的大段控制）
            try:
                if not self.get_ready:
                    # 机器狗整体初始化中
                    pass

                # 第一次直线行驶
                elif not self.first_straight and rgb is not None:
                    print("第一次直线行驶")
                    # PID 控制初始化
                    if not self.pid_used:
                        self.pid = PID(
                            kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0)
                        )
                        self.pid_used = True
                    self.motioncontroller.cmd_msg.motion_id = 308
                    self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
                    self.start_flag_timer("first_straight", 5.0, True)
                    self.start_flag_timer("pid_used", 5.0, False)

                    deviation = detect_deviation(rgb)
                    base_speed = 0.2
                    correction = 1.5 * self.pid.calculate(0.0, deviation)
                    print("偏航调节系数：", correction)
                    self.motioncontroller.cmd_msg.vel_des = [
                        base_speed,
                        0.0,
                        base_speed * correction,
                    ]

                # 第一次直角转弯
                elif not self.first_turn:
                    print("第一次直角转弯")
                    self.motioncontroller.cmd_msg.motion_id = 308
                    self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
                    self.motioncontroller.cmd_msg.vel_des = [0.0, 0.0, -0.61]
                    self.start_flag_timer("first_turn", 3.0, True)

                # 第二次直线行驶
                elif not self.second_straight and rgb is not None:
                    print("第二次直线行驶")
                    self.motioncontroller.cmd_msg.motion_id = 308
                    self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]

                    if not self.distance_detector_used:
                        self.distance_detector = LineDistanceDetector(
                            roi_width=20, smooth_window=5
                        )
                        self.distance_detector_used = True
                    if not self.pid_used:
                        self.pid = PID(
                            kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0)
                        )
                        self.pid_used = True

                    deviation = detect_deviation(rgb)
                    base_speed = 0.2
                    correction = 1.5 * self.pid.calculate(0.0, deviation)
                    print("偏航调节系数：", correction)
                    self.motioncontroller.cmd_msg.vel_des = [
                        base_speed,
                        0.0,
                        base_speed * correction,
                    ]

                    distance = self.distance_detector.detect_line_distance(rgb)
                    print("底线距离：", distance)
                    if distance < 120:
                        self.distance_detector_used = False
                        self.pid_used = False
                        self.second_straight = True

                # 文本识别段（使用 AI 相机帧 ai）
                elif self.second_straight and not self.text1_result and ai is not None:
                    print("第一次文本识别")
                    # 进入识别序列：先起步抖动2秒，再识别
                    if self.text1_get:
                        self.motioncontroller.cmd_msg.motion_id = 308
                        self.motioncontroller.cmd_msg.step_height = [0.06, 0.06]
                        self.motioncontroller.cmd_msg.vel_des = [
                            (random.random() - 0.5),
                            0.0,
                            0.0,
                        ]
                    else:
                        self.motioncontroller.cmd_msg.motion_id = 111
                        self.text1_result = text_detector(ai)
                        self.text1_get = True
                        self.start_flag_timer("text1_get", 2.0, False)

                # 第一次角度调节
                elif not self.first_adjust:
                    print("第一次角度调节")
                    # PID 控制初始化
                    if not self.pid_used:
                        self.pid = PID(
                            kp=0.6, ki=0.0, kd=0.4, output_limits=(-1.0, 1.0)
                        )
                        self.pid_used = True
                    self.start_flag_timer("pid_used", 3.0, False)
                    self.start_flag_timer("first_adjust", 3.0, True)

                    self.motioncontroller.cmd_msg.motion_id = 308
                    deviation = compute_line_offset(rgb)
                    print("偏航角:", deviation)

                    base_speed = 0.3
                    correction = self.pid.calculate(0.0, deviation)
                    print("比例系数:", correction)
                    self.motioncontroller.cmd_msg.rpy_des = [0.0, 0.3, 0.0]
                    self.motioncontroller.cmd_msg.vel_des = [
                        0.0,
                        0.0,
                        base_speed * correction,
                    ]

                elif not self.first_fix:
                    print("机器狗前进补正")
                    if not self.distance_detector_used:
                        self.distance_detector = LineDistanceDetector(
                            roi_width=20, smooth_window=5
                        )
                        self.distance_detector_used = True

                    self.motioncontroller.cmd_msg.vel_des = [0.2, 0.0, 0.0]
                    # 距离检查
                    distance = self.distance_detector.detect_line_distance(rgb)
                    print("当前距离为", distance)
                    if distance < 30:
                        self.distance_detector_used = False
                        self.first_fix = True

                elif self.text1_result:
                    print("程序结束")
                    # 停止
                    self.motioncontroller.cmd_msg.motion_id = 101
                    time.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f"控制状态机异常: {e}")

            # 周期控制与节流日志
            elapsed = time.perf_counter() - t0
            sleep_t = max(0.0, dt - elapsed)
            time.sleep(sleep_t)

    # ----------- 线程级一次性计时器（不依赖 ROS2 Timer） -----------
    def start_flag_timer(
        self, flag_name: str, duration_sec: float, target: bool = True
    ):
        """
        threading.Timer 版本：duration_sec 秒后将 self.flag_name 置为 target。
        如果已有同名定时器在运行，则直接忽略（避免反复设置）。
        """
        if flag_name in self.active_timers:
            return

        def timer_callback():
            try:
                setattr(self, flag_name, target)
                self.get_logger().info(f"{flag_name} 已置为 {target}")
            finally:
                # 清理记录
                self.active_timers.pop(flag_name, None)

        t = threading.Timer(duration_sec, timer_callback)
        self.active_timers[flag_name] = t
        t.daemon = True
        t.start()

    # ----------- 资源清理 -----------
    def cleanup(self):
        self._stop_event.set()

        # 停止运动控制线程
        try:
            if hasattr(self, "motioncontroller"):
                self.motioncontroller.stop_thread()
        except Exception as e:
            self.get_logger().warn(f"停止运动控制线程异常: {e}")

        # 终止所有计时器
        for name, tm in list(self.active_timers.items()):
            try:
                tm.cancel()
            except Exception:
                pass
        self.active_timers.clear()

        # 停止相机发布
        try:
            if self.client and self.client.service_is_ready():
                stop_request = CameraService.Request()
                stop_request.command = CameraService.Request.STOP_IMAGE_PUBLISH
                self.client.call_async(stop_request)
                self.get_logger().info("发送最终停止相机请求")
        except Exception as e:
            self.get_logger().error(f"停止相机失败: {e}")

        # 销毁 OpenCV 窗口
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()

    # MultiThreadedExecutor：保证订阅/服务/misc 可并行
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断")
    finally:
        try:
            node.cleanup()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
