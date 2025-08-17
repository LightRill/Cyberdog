# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import subprocess


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


# class RGBImageSubscriber(Node):
#     def __init__(self):
#         super().__init__("rgb_image_viewer", namespace=get_namespace())
#         self.bridge = CvBridge()

#         # 创建订阅者，订阅/image_rgb话题
#         self.subscription = self.create_subscription(
#             Image,
#             "/image_rgb",  # 确保话题名称与相机发布的一致
#             self.image_callback,
#             10,  # 队列大小
#         )
#         self.subscription  # 防止未使用变量警告
#         self.get_logger().info("RGB图像查看器已启动，等待图像数据...")

#         # 创建OpenCV窗口
#         cv2.namedWindow("RGB Camera Feed", cv2.WINDOW_NORMAL)
#         cv2.resizeWindow("RGB Camera Feed", 800, 600)

#     def image_callback(self, msg):
#         try:
#             # 将ROS图像消息转换为OpenCV格式
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#             cv2.imshow("RGB Camera Feed", cv_image)
#             cv2.waitKey(1)  # 需要处理OpenCV事件

#         except Exception as e:
#             self.get_logger().error(f"图像转换错误: {str(e)}")

#     def destroy_node(self):
#         cv2.destroyAllWindows()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     viewer = RGBImageSubscriber()

#     try:
#         rclpy.spin(viewer)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         viewer.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import math
# import time
# import subprocess


# # ----------------- PID 控制器 -----------------
# class PID:
#     def __init__(
#         self, kp=0.6, ki=0.0, kd=0.2, output_limits=(-1.0, 1.0), integral_limit=None
#     ):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.output_limits = output_limits
#         self.integral_limit = integral_limit
#         self.error_sum = 0.0
#         self.last_error = 0.0
#         self.last_time = time.perf_counter()

#     def reset(self):
#         self.error_sum = 0.0
#         self.last_error = 0.0
#         self.last_time = time.perf_counter()

#     def calculate(self, setpoint, measurement):
#         now = time.perf_counter()
#         dt = now - self.last_time
#         if dt <= 0.0:
#             dt = 1e-6
#         error = setpoint - measurement
#         self.error_sum += error * dt
#         if self.integral_limit is not None:
#             self.error_sum = max(
#                 min(self.error_sum, self.integral_limit), -self.integral_limit
#             )
#         error_delta = (error - self.last_error) / dt
#         output = (
#             (self.kp * error) + (self.ki * self.error_sum) + (self.kd * error_delta)
#         )
#         output = max(min(output, self.output_limits[1]), self.output_limits[0])
#         self.last_error = error
#         self.last_time = now
#         return output


# # ----------------- 偏航检测 -----------------
# def safe_fitline(points):
#     """安全处理 fitLine 返回类型"""
#     line_params = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
#     if isinstance(line_params, list) and len(line_params) == 4:
#         vx = (
#             line_params[0][0] if hasattr(line_params[0], "__iter__") else line_params[0]
#         )
#         vy = (
#             line_params[1][0] if hasattr(line_params[1], "__iter__") else line_params[1]
#         )
#         x0 = (
#             line_params[2][0] if hasattr(line_params[2], "__iter__") else line_params[2]
#         )
#         y0 = (
#             line_params[3][0] if hasattr(line_params[3], "__iter__") else line_params[3]
#         )
#         return vx, vy, x0, y0
#     return line_params


# def detect_deviation(cv_image, visualize=False):
#     """检测图像的偏航角度，并可视化调试"""
#     h, w = cv_image.shape[:2]
#     max_dimension = 800
#     scale = max_dimension / max(h, w)
#     nw, nh = int(w * scale), int(h * scale)
#     img = cv2.resize(cv_image, (nw, nh), interpolation=cv2.INTER_AREA)

#     # ---- 自适应 ROI 掩码 ----
#     mask = np.zeros((nh, nw), dtype=np.uint8)
#     bw = nw * 0.8  # 宽路适应
#     tw = nw * 0.1
#     hr = 0.15
#     pts = np.array(
#         [
#             [(nw - bw) // 2, nh],
#             [(nw + bw) // 2, nh],
#             [(nw + tw) // 2, int(nh * (1 - hr))],
#             [(nw - tw) // 2, int(nh * (1 - hr))],
#         ],
#         np.int32,
#     )
#     cv2.fillPoly(mask, [pts], 255)

#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([40, 255, 255]))
#     masked = cv2.bitwise_and(yellow_mask, yellow_mask, mask=mask)

#     kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
#     proc = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel)

#     cnts, _ = cv2.findContours(proc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     valid = [c for c in cnts if cv2.contourArea(c) > 20]

#     deviation_angle = 0.0
#     if valid:
#         pts_all = np.vstack([c.squeeze() for c in valid])
#         miny, maxy = pts_all[:, 1].min(), pts_all[:, 1].max()
#         centers = []
#         step = max(1, (maxy - miny) // 50)
#         for y in range(int(miny), int(maxy), step):
#             row = pts_all[np.abs(pts_all[:, 1] - y) < 0.5]
#             if len(row) > 1:
#                 left = row[:, 0].min()
#                 right = row[:, 0].max()
#                 centers.append([(left + right) / 2, y])
#         cp = np.array(centers)
#         if cp.size:
#             bc = tuple(cp[cp[:, 1].argmax()])
#             start_y = max(miny, bc[1] - 30)
#             bp = cp[cp[:, 1] >= start_y]
#             if len(bp) > 1:
#                 vx, vy, x0, y0 = map(float, safe_fitline(bp))
#                 ang = math.degrees(math.atan2(vy, vx) - math.pi / 2)
#                 if ang > 90:
#                     ang -= 180
#                 if ang < -90:
#                     ang += 180
#                 deviation_angle = ang

#                 # 可视化中心点与拟合线
#                 if visualize:
#                     for pt in bp:
#                         cv2.circle(img, (int(pt[0]), int(pt[1])), 2, (0, 0, 255), -1)
#                     for y in range(int(miny), int(maxy), 5):
#                         x_line = int(((y - y0) / vy) * vx + x0)
#                         cv2.circle(img, (x_line, y), 1, (0, 255, 0), -1)
#     return deviation_angle, img


# # ----------------- ROS2 订阅节点 -----------------
# def get_namespace():
#     """自动检测 ROS2 命名空间"""
#     import subprocess

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


# class RGBImageSubscriber(Node):
#     def __init__(self):
#         super().__init__("rgb_image_viewer", namespace=get_namespace())
#         self.bridge = CvBridge()
#         self.subscription = self.create_subscription(
#             Image, "/image_rgb", self.image_callback, 10
#         )
#         self.pid = PID(kp=0.6, ki=0.0, kd=0.2, output_limits=(-1, 1))
#         self.base_speed = 0.5
#         cv2.namedWindow("RGB Camera Feed", cv2.WINDOW_NORMAL)
#         cv2.resizeWindow("RGB Camera Feed", 800, 600)
#         self.get_logger().info("RGB图像查看器已启动，等待图像数据...")

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             deviation, vis_img = detect_deviation(cv_image, visualize=True)
#             correction = self.pid.calculate(0.0, deviation)
#             left_speed = self.base_speed - 0.5 * correction
#             right_speed = self.base_speed + 0.5 * correction
#             # 打印调试信息
#             print(
#                 f"偏航角: {deviation:.2f}, correction: {correction:.2f}, 左轮: {left_speed:.2f}, 右轮: {right_speed:.2f}"
#             )
#             # 显示可视化
#             cv2.imshow("RGB Camera Feed", vis_img)
#             cv2.waitKey(1)
#         except Exception as e:
#             self.get_logger().error(f"图像转换错误: {str(e)}")

#     def destroy_node(self):
#         cv2.destroyAllWindows()
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     viewer = RGBImageSubscriber()
#     try:
#         rclpy.spin(viewer)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         viewer.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess


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


def detect_line_distance(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    height, width = mask.shape
    mid_x = width // 2

    for y in range(height - 1, 0, -1):
        if mask[y, mid_x] == 255:
            return height - y

    return height


def compute_line_offset(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    height, width = mask.shape
    mid_x = width // 2

    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=20
    )

    if lines is not None:
        # 取最长的线
        longest = max(
            lines,
            key=lambda line: np.linalg.norm(
                (line[0][0] - line[0][2], line[0][1] - line[0][3])
            ),
        )
        x1, y1, x2, y2 = longest[0]

        # 画线
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 计算角度误差
        angle = np.arctan2(y2 - y1, x2 - x1)  # 弧度
        angle_deg = np.degrees(angle)

        return angle_deg
    else:
        return 0


class DebugLineDetector(Node):
    def __init__(self):
        super().__init__("line_debug_viewer", namespace=get_namespace())
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/image_rgb",
            self.image_callback,
            10,
        )

        cv2.namedWindow("Line Debug View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Line Debug View", 800, 600)

        self.get_logger().info("Line Debug Viewer 已启动，等待图像数据...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 调用检测函数
            distance = detect_line_distance(cv_image)
            offset_error = compute_line_offset(cv_image)

            # 在图像上显示结果
            cv2.putText(
                cv_image,
                f"Distance: {distance}",
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                cv_image,
                f"Offset Error: {offset_error}",
                (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Line Debug View", cv_image)
            cv2.waitKey(1)

            # 控制台打印
            self.get_logger().info(f"Distance={distance}, Offset={offset_error}")

        except Exception as e:
            self.get_logger().error(f"图像处理错误: {str(e)}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DebugLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
