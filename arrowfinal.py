import cv2
import numpy as np

def detect_arrow_direction(image_path):
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        return "无法读取图像"
    
    # 转换为HSV颜色空间（更容易检测特定颜色）
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # 定义绿色的HSV范围
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    
    # 创建绿色掩膜
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # 寻找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return "未检测到绿色箭头"
    
    # 找到最大的轮廓（假设箭头是最大的绿色物体）
    largest_contour = max(contours, key=cv2.contourArea)
    
    # 计算轮廓的凸包
    hull = cv2.convexHull(largest_contour, returnPoints=False)
    
    # 寻找凸包缺陷（用于检测箭头尖端）
    defects = cv2.convexityDefects(largest_contour, hull)
    
    if defects is None or len(defects) < 3:
        return "无法确定箭头方向"
    
    # 找到最深的凸包缺陷（假设这是箭头尖端）
    deepest_defect = max(defects, key=lambda x: x[0, 3])
    start_idx, end_idx, far_idx, _ = deepest_defect[0]
    
    # 获取尖端点的坐标
    tip = tuple(largest_contour[far_idx][0])
    
    # 获取箭头的起点和终点
    start_point = tuple(largest_contour[start_idx][0])
    end_point = tuple(largest_contour[end_idx][0])
    
    # 计算箭头的方向
    if start_point[0] < end_point[0]:
        # 如果起点在终点的左侧，则箭头指向右
        if tip[0] > (start_point[0] + end_point[0]) / 2:
            return "右"
        else:
            return "左"
    else:
        # 如果起点在终点的右侧，则箭头指向左
        if tip[0] < (start_point[0] + end_point[0]) / 2:
            return "左"
        else:
            return "右"

# 使用示例
image_path = "./imgs/left.jpg"  # 替换为你的图片路径
direction = detect_arrow_direction(image_path)
print(f"箭头方向: {direction}")
