#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from protocol.srv import CameraService
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import subprocess
import uuid

from qrcode_text_detector import main_detection


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


class AICameraClient(Node):
    def __init__(self):
        super().__init__("ai_camera_client", namespace=get_namespace())

        self.client = self.create_client(CameraService, "camera_service")
        self.get_logger().info("等待相机服务上线...")

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("服务未找到，重试中...")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.window_open = False
        self.subscription = None
        self.get_logger().info("相机服务已连接")

    def start_camera(self):
        """启动相机前先尝试停止相机，防止重复运行失败"""
        self.stop_camera(lambda _: self._do_start_camera())

    def _do_start_camera(self):
        request = CameraService.Request()
        request.command = CameraService.Request.START_IMAGE_PUBLISH
        request.height = 640
        request.width = 480
        request.fps = 30

        future = self.client.call_async(request)
        future.add_done_callback(self._camera_start_callback)

    def stop_camera(self, callback=None):
        """安全停止相机"""
        try:
            request = CameraService.Request()
            request.command = CameraService.Request.STOP_IMAGE_PUBLISH
            future = self.client.call_async(request)

            def _stop_done(fut):
                try:
                    fut.result()
                    self.get_logger().info("相机停止完成")
                except Exception as e:
                    self.get_logger().error(f"停止相机失败: {e}")
                if callback:
                    callback(fut)

            future.add_done_callback(_stop_done)
        except Exception as e:
            self.get_logger().error(f"停止相机异常: {e}")
            if callback:
                callback(None)

    def _camera_start_callback(self, future):
        try:
            response = future.result()
            if response.result == CameraService.Response.RESULT_SUCCESS:
                self.get_logger().info("相机启动成功，开始订阅图像")
                if not self.subscription:
                    self.subscription = self.create_subscription(
                        Image, "image", self._image_callback, 10
                    )
                threading.Thread(target=self._display_thread, daemon=True).start()
            else:
                self.get_logger().error("相机启动失败")
        except Exception as e:
            self.get_logger().error(f"服务调用失败: {e}")

    def _image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换错误: {e}")

    def _display_thread(self):
        self.window_open = True
        cv2.destroyAllWindows()  # 确保重复运行时窗口关闭
        cv2.namedWindow("AI Camera Feed", cv2.WINDOW_NORMAL)

        while self.window_open and rclpy.ok():
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
                cv2.imshow("AI Camera Feed", frame)
                result = main_detection(frame)

                if result:
                    display_text = f"识别结果: {result}"
                    color = (0, 255, 0)
                else:
                    display_text = "未识别"
                    color = (0, 0, 255)

                # 显示识别结果
                cv2.putText(
                    frame,
                    display_text,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    color,
                    2,
                )
            if cv2.waitKey(20) & 0xFF == ord("q"):
                self.window_open = False
                break

        cv2.destroyAllWindows()
        self.get_logger().info("关闭相机显示")


def main(args=None):
    rclpy.init(args=args)
    camera_client = AICameraClient()

    try:
        camera_client.start_camera()
        rclpy.spin(camera_client)
    except KeyboardInterrupt:
        pass
    finally:
        camera_client.stop_camera()
        camera_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from protocol.srv import CameraService
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import threading
# import subprocess

# # ============ 引入二维码和文本检测函数 ============
# # 假设你把之前的 detection 代码放在 detection.py 里
# from qrcode_text_detector import main_detection


# def get_namespace():
#     """自动检测 ROS2 命名空间"""
#     try:
#         result = subprocess.run(
#             ["ros2", "node", "list"],
#             stdout=subprocess.PIPE,
#             stderr=subprocess.PIPE,
#             universal_newlines=True,
#             check=True,
#         )
#         for line in result.stdout.splitlines():
#             if "mi_" in line:
#                 parts = line.strip().split("/")
#                 if len(parts) > 1:
#                     return "/" + parts[1]
#     except subprocess.CalledProcessError as e:
#         print("获取命名空间失败:", e)
#     return "/"  # 默认根命名空间


# class AICameraClient(Node):
#     def __init__(self):
#         super().__init__("ai_camera_client", namespace=get_namespace())
#         self.client = self.create_client(CameraService, "camera_service")
#         self.get_logger().info("等待相机服务上线...")

#         while not self.client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().warn("服务未找到，重试中...")

#         self.bridge = CvBridge()
#         self.latest_frame = None
#         self.window_open = False
#         self.get_logger().info("相机服务已连接")

#     def start_camera(self):
#         """启动相机并开始订阅图像"""
#         request = CameraService.Request()
#         request.command = CameraService.Request.START_IMAGE_PUBLISH
#         request.height = 640
#         request.width = 480
#         request.fps = 30

#         future = self.client.call_async(request)
#         future.add_done_callback(self._camera_start_callback)

#     def _camera_start_callback(self, future):
#         """相机启动回调"""
#         try:
#             response = future.result()
#             if response.result == CameraService.Response.RESULT_SUCCESS:
#                 self.get_logger().info("相机启动成功，开始订阅图像")
#                 self.subscription = self.create_subscription(
#                     Image, "image", self._image_callback, 10
#                 )
#                 threading.Thread(target=self._display_thread, daemon=True).start()
#             else:
#                 self.get_logger().error("相机启动失败")
#         except Exception as e:
#             self.get_logger().error(f"服务调用失败: {str(e)}")

#     def _image_callback(self, msg):
#         """图像数据回调"""
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             self.latest_frame = cv_image
#         except Exception as e:
#             self.get_logger().error(f"图像转换错误: {str(e)}")

#     def _display_thread(self):
#         """OpenCV显示线程"""
#         self.window_open = True
#         cv2.namedWindow("AI Camera Feed", cv2.WINDOW_NORMAL)

#         while self.window_open and rclpy.ok():
#             if self.latest_frame is not None:
#                 frame = self.latest_frame.copy()

#                 # ============ 调用检测函数 ============
#                 result = main_detection(frame)

#                 if result:
#                     display_text = f"识别结果: {result}"
#                     color = (0, 255, 0)
#                 else:
#                     display_text = "未识别"
#                     color = (0, 0, 255)

#                 # 显示识别结果
#                 cv2.putText(
#                     frame,
#                     display_text,
#                     (10, 60),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     1,
#                     color,
#                     2,
#                 )

#                 # 额外调试信息
#                 cv2.putText(
#                     frame,
#                     "CyberDog AI Camera",
#                     (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     1,
#                     (255, 255, 0),
#                     2,
#                 )
#                 cv2.putText(
#                     frame,
#                     "Press 'Q' to exit",
#                     (10, frame.shape[0] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX,
#                     0.7,
#                     (0, 0, 255),
#                     2,
#                 )

#                 cv2.imshow("AI Camera Feed", frame)

#             if cv2.waitKey(20) & 0xFF == ord("q"):
#                 self.window_open = False
#                 break

#         cv2.destroyAllWindows()
#         self.get_logger().info("关闭相机显示")


# def main(args=None):
#     rclpy.init(args=args)
#     camera_client = AICameraClient()

#     try:
#         camera_client.start_camera()
#         rclpy.spin(camera_client)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         camera_client.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
