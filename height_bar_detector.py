import cv2
import numpy as np


def detect_red_pole(image):
    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义更严格的红色HSV范围
    lower_red1 = np.array([0, 170, 130])  # 高饱和度和亮度要求
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 170, 130])
    upper_red2 = np.array([180, 255, 255])

    # 创建红色掩模
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # 形态学操作 - 只使用开运算去除噪点
    kernel = np.ones((5, 5), np.uint8)
    processed = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    # 寻找轮廓
    contours, _ = cv2.findContours(
        processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    # 候选轮廓列表（综合考虑长宽比和位置）
    candidate_contours = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:  # 忽略小面积轮廓
            continue

        # 获取边界矩形
        x, y, w, h = cv2.boundingRect(cnt)

        # 长宽比筛选（杆子通常是细长的）
        aspect_ratio = max(w, h) / min(w, h)

        # 位置筛选：杆子通常在上半部分的中央位置
        height_ratio = y / image.shape[0]  # 上部区域的轮廓可能性更高

        # 积分权重计算（长宽比越高、位置越高，分数越高）
        score = aspect_ratio * 0.7 + (1 - height_ratio) * 0.3

        candidate_contours.append((cnt, score, x, y, w, h))

    # 如果没有找到候选轮廓，则返回None
    if not candidate_contours:
        return None

    # 根据综合得分排序（得分高的优先）
    candidate_contours.sort(key=lambda x: x[1], reverse=True)

    # 返回得分最高的轮廓（最符合杆子特征）
    return candidate_contours[0][0]


# 距离计算函数不变
def convert_to_cm(box_height):
    if box_height <= 0:
        return float("inf")

    k1 = 90 * 147
    k2 = 30 * 590
    k = (k1 + k2) / 2

    cm_distance = k / box_height
    return max(cm_distance, 0)


def height_bar_check(cv_image):
    if cv_image is None:
        print("错误：无法读取图像")
        return

    height, width = cv_image.shape[:2]

    # 检测红色杆子
    contour = detect_red_pole(cv_image)

    if contour is not None:
        # 获取边界框
        x, y, w, h = cv2.boundingRect(contour)

        # print(f"[检测结果] 矩形框高度 h = {h} 像素")

        cm_distance = convert_to_cm(h)
        return cm_distance
    else:
        return -1
