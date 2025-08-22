# import cv2
# import numpy as np
# import math
# import rclpy
# from rclpy.node import Node
# from protocol.srv import CameraService
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import threading
# import subprocess


# def detect_yellow_light_distance(img):
#     if img is None:
#         return None, None, None
#     # 获取图像尺寸
#     height, width = img.shape[:2]

#     # 光照自适应
#     img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     img_hsv[:, :, 2] = cv2.equalizeHist(img_hsv[:, :, 2])
#     img_eq = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

#     # 转换为HSV
#     hsv = cv2.cvtColor(img_eq, cv2.COLOR_BGR2HSV)

#     # 黄色范围
#     lower_yellow1 = np.array([20, 60, 100])
#     upper_yellow1 = np.array([35, 255, 255])
#     lower_yellow2 = np.array([15, 40, 80])
#     upper_yellow2 = np.array([40, 255, 255])

#     mask1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)
#     mask2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)
#     mask = cv2.bitwise_or(mask1, mask2)

#     # 形态学
#     kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
#     kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

#     # 轮廓检测
#     contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

#     largest_circle = None
#     max_area = 0

#     for contour in contours:
#         area = cv2.contourArea(contour)
#         if area < 5000:
#             continue

#         perimeter = cv2.arcLength(contour, True)
#         if perimeter == 0:
#             continue

#         circularity = (4 * np.pi * area) / (perimeter**2)
#         if circularity > 0.4:
#             (x, y), radius = cv2.minEnclosingCircle(contour)
#             center = (int(x), int(y))
#             radius = int(radius)
#             diameter_pixels = 2 * radius

#             if area > max_area:
#                 max_area = area
#                 largest_circle = (center, radius, area, diameter_pixels)

#     distance = None
#     if largest_circle:
#         detected_area = largest_circle[2]
#         k1 = 100 * math.sqrt(33204.0)
#         k2 = 40 * math.sqrt(253766.0)
#         k = (k1 + k2) / 2
#         distance = k / math.sqrt(detected_area)
#         distance = max(distance, 0)

#     return largest_circle, distance, img


# def get_namespace():
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
#     return "/"


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
#         self.subscription = None
#         self.get_logger().info("相机服务已连接")

#     def start_camera(self):
#         self.stop_camera(lambda _: self._do_start_camera())

#     def _do_start_camera(self):
#         request = CameraService.Request()
#         request.command = CameraService.Request.START_IMAGE_PUBLISH
#         request.height = 640
#         request.width = 480
#         request.fps = 30

#         future = self.client.call_async(request)
#         future.add_done_callback(self._camera_start_callback)

#     def stop_camera(self, callback=None):
#         try:
#             request = CameraService.Request()
#             request.command = CameraService.Request.STOP_IMAGE_PUBLISH
#             future = self.client.call_async(request)

#             def _stop_done(fut):
#                 try:
#                     fut.result()
#                     self.get_logger().info("相机停止完成")
#                 except Exception as e:
#                     self.get_logger().error(f"停止相机失败: {e}")
#                 if callback:
#                     callback(fut)

#             future.add_done_callback(_stop_done)
#         except Exception as e:
#             self.get_logger().error(f"停止相机异常: {e}")
#             if callback:
#                 callback(None)

#     def _camera_start_callback(self, future):
#         try:
#             response = future.result()
#             if response.result == CameraService.Response.RESULT_SUCCESS:
#                 self.get_logger().info("相机启动成功，开始订阅图像")
#                 if not self.subscription:
#                     self.subscription = self.create_subscription(
#                         Image, "image", self._image_callback, 10
#                     )
#                 threading.Thread(target=self._display_thread, daemon=True).start()
#             else:
#                 self.get_logger().error("相机启动失败")
#         except Exception as e:
#             self.get_logger().error(f"服务调用失败: {e}")

#     def _image_callback(self, msg):
#         try:
#             self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except Exception as e:
#             self.get_logger().error(f"图像转换错误: {e}")

#     def _display_thread(self):
#         self.window_open = True
#         cv2.destroyAllWindows()
#         cv2.namedWindow("AI Camera Feed", cv2.WINDOW_NORMAL)

#         while self.window_open and rclpy.ok():
#             if self.latest_frame is not None:
#                 frame = self.latest_frame.copy()
#                 circle, distance, _ = detect_yellow_light_distance(frame)
#                 if circle:
#                     (x, y), r, area, diam = circle
#                     cv2.circle(frame, (x, y), r, (0, 255, 255), 2)
#                     if distance is not None:
#                         cv2.putText(
#                             frame,
#                             f"Dist: {distance:.1f} cm",
#                             (x - 40, y - r - 10),
#                             cv2.FONT_HERSHEY_SIMPLEX,
#                             0.6,
#                             (0, 0, 255),
#                             2,
#                         )
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
#         camera_client.stop_camera()
#         camera_client.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import cv2
import numpy as np
import threading
import subprocess
import rclpy
from rclpy.node import Node
from protocol.srv import CameraService
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def detect_red_pole(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 170, 130])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 170, 130])
    upper_red2 = np.array([180, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    red_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    # mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    # red_mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    processed = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    candidate_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = max(w, h) / min(w, h)
        height_ratio = y / image.shape[0]
        score = aspect_ratio * 0.7 + (1 - height_ratio) * 0.3
        candidate_contours.append((cnt, score, x, y, w, h))

    if not candidate_contours:
        return None

    candidate_contours.sort(key=lambda x: x[1], reverse=True)
    return candidate_contours[0][0]


def convert_to_cm(box_height):
    if box_height <= 0:
        return float("inf")
    k1 = 90 * 147
    k2 = 30 * 590
    k = (k1 + k2) / 2
    cm_distance = k / box_height
    return max(cm_distance, 0)


def get_namespace():
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
    return "/"


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
        cv2.destroyAllWindows()
        cv2.namedWindow("AI Camera Feed", cv2.WINDOW_NORMAL)

        while self.window_open and rclpy.ok():
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()

                # 检测红色杆子
                contour = detect_red_pole(frame)
                if contour is not None:
                    x, y, w, h = cv2.boundingRect(contour)
                    cm_distance = convert_to_cm(h)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        f"{cm_distance:.1f} cm",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                    )

                cv2.imshow("AI Camera Feed", frame)

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
