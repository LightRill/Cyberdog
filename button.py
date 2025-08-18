from PIL import Image
import numpy as np


def dark_button(cv_image, threshold=60, dark_ratio=0.5):
    """
    判断图片是否较暗
    :param image_path: 图片路径
    :param threshold: 暗像素的阈值（0-255），值越小越暗
    :param dark_ratio: 暗像素占比阈值
    :return: 如果暗像素比例超过阈值返回True，否则False
    """

    # 将图片转换为numpy数组
    img_array = np.array(cv_image)

    # 计算每个像素的亮度（加权平均值）
    # 权重参考人眼感知：0.299*R + 0.587*G + 0.114*B
    brightness = np.dot(img_array[..., :3], [0.299, 0.587, 0.114])

    # 计算暗像素比例（亮度低于阈值的像素）
    dark_pixels = np.sum(brightness < threshold)
    total_pixels = brightness.size
    dark_percentage = dark_pixels / total_pixels

    return dark_percentage >= dark_ratio


# 示例用法
if __name__ == "__main__":
    image_path = "./imgs/2222.jpg"  # 替换为你的图片路径
    result = dark_button(image_path)
    print(f"图片是否较暗？ {result}")
