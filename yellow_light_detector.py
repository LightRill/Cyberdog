import cv2
import numpy as np
import math


def detect_yellow_light_distance(img):
    if img is None:
        print("Error: 无效的图像输入")
        return None, None, None
    # 获取图像尺寸
    height, width = img.shape[:2]

    # 增加光照自适应处理
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_hsv[:, :, 2] = cv2.equalizeHist(img_hsv[:, :, 2])
    img_eq = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

    # 转换为HSV色彩空间
    hsv = cv2.cvtColor(img_eq, cv2.COLOR_BGR2HSV)

    # 扩展黄色范围
    lower_yellow1 = np.array([20, 60, 100])
    upper_yellow1 = np.array([35, 255, 255])
    lower_yellow2 = np.array([15, 40, 80])
    upper_yellow2 = np.array([40, 255, 255])

    # 创建双黄色范围掩膜并组合
    mask1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)
    mask2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 增强形态学操作
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

    # 轮廓检测
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # 寻找最大的黄色圆形
    largest_circle = None
    max_area = 0
    diameter_pixels = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 5000:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue

        circularity = (4 * np.pi * area) / (perimeter**2)

        if circularity > 0.4:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            diameter_pixels = 2 * radius

            if area > max_area:
                max_area = area
                largest_circle = (center, radius, area, diameter_pixels)

    # 计算到黄灯的距离
    distance = None
    vertical_position = None

    if largest_circle:
        # 获取检测到的面积
        detected_area = area

        k1 = 100 * math.sqrt(33204.0)
        k2 = 40 * math.sqrt(253766.0)
        k = (k1 + k2) / 2
        distance = k / math.sqrt(detected_area)

        # 确保距离不为负
        distance = max(distance, 0)

    return largest_circle, distance, img


def yellow_light_check(cv_img):
    result, distance, result_img = detect_yellow_light_distance(cv_img)

    if result:
        (x, y), r, area, diam = result
        print(
            f"检测到最大的黄灯: 位置=({x}, {y}), 半径={r}px, 直径={diam}px, 面积={area}px²"
        )
        if distance is not None:
            # print(f"计算距离: {distance:.2f} 厘米")
            return distance
        else:
            print("距离计算失败：需要校准相机参数")
    else:
        print("未检测到黄灯")
        return -1
