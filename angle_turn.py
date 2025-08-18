import cv2
import numpy as np

from collections import deque


import cv2
import numpy as np
from collections import deque


class LineDistanceDetector:
    def __init__(
        self, roi_width=20, smooth_window=5, jump_threshold=None, ignore_frames=0
    ):
        self.roi_width = roi_width
        self.smooth_window = smooth_window
        self.distance_buffer = deque(maxlen=smooth_window)
        self.last_distance = None
        self.jump_threshold = jump_threshold
        self.ignore_frames = ignore_frames
        self.cur_frame = 0

    def detect_line_distance(self, image):
        """
        检测图像中黄色线条距离底部的距离，使用竖带质心法并平滑输出。
        如果与上次结果跳变过大，则返回 0。
        :param image: BGR图像
        :return: 平滑后的距离（像素）或 0
        """

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        height, width = mask.shape
        mid_x = width // 2

        # 忽略最开始的帧
        self.cur_frame += 1
        if self.cur_frame <= self.ignore_frames:
            return self.last_distance if self.last_distance is not None else height

        # 取中间竖带区域
        x1 = max(mid_x - self.roi_width // 2, 0)
        x2 = min(mid_x + self.roi_width // 2, width)
        roi = mask[:, x1:x2]

        # 计算质心
        M = cv2.moments(roi)
        if M["m00"] > 0:
            cy = int(M["m01"] / M["m00"])
            distance = height - cy
        else:
            distance = height  # 未检测到线条，返回最大距离

        # 滑动平均平滑
        self.distance_buffer.append(distance)
        smooth_distance = sum(self.distance_buffer) / len(self.distance_buffer)

        # 跳变检测
        if (
            self.jump_threshold is not None
            and self.last_distance is not None
            and distance - self.last_distance > self.jump_threshold
        ):
            result = 0
        else:
            result = smooth_distance

        # 更新历史值
        self.last_distance = smooth_distance

        return result


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
