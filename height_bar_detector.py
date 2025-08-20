import cv2
import numpy as np


# 红色杆子检测函数
def detect_red_pole(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 红色范围
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # 掩模
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # 形态学处理
    kernel = np.ones((5, 5), np.uint8)
    processed = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    processed = cv2.morphologyEx(processed, cv2.MORPH_CLOSE, kernel)

    # 轮廓
    contours, _ = cv2.findContours(
        processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    max_contour = None
    max_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > max_area:
            max_area = area
            max_contour = cnt

    return max_contour


# 像素高度 → 距离（cm）
def convert_to_cm(box_height):
    if box_height <= 0:
        return float("inf")

    k1 = 90 * 147
    k2 = 30 * 590
    k = (k1 + k2) / 2

    return max(k / box_height, 0)


# 主函数
def height_bar_check(cv_image):
    contour = detect_red_pole(cv_image)

    if contour is not None:
        _, _, _, h = cv2.boundingRect(contour)
        return convert_to_cm(h)
    else:
        return -1


if __name__ == "__main__":
    image_path = "./imgs/red30.jpg"
    image = cv2.imread(image_path)
    distance_cm = height_bar_check(image)
    print(distance_cm)
