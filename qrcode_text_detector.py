import cv2
import numpy as np
import math
from pyzbar.pyzbar import decode

# ============ 二维码识别部分 ============
# 图像增强参数
ENHANCE_PARAMS = {
    "clahe_clip": 3.0,
    "denoise_strength": 10,
    "scale_factors": [1.0, 0.8, 1.2],
    "adaptive_block": 51,
}


def preprocess_image(img):
    """图像预处理流水线"""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=ENHANCE_PARAMS["clahe_clip"], tileGridSize=(8, 8))
    enhanced_l = clahe.apply(l)

    enhanced = cv2.cvtColor(cv2.merge((enhanced_l, a, b)), cv2.COLOR_LAB2BGR)
    denoised = cv2.fastNlMeansDenoisingColored(
        enhanced,
        None,
        ENHANCE_PARAMS["denoise_strength"],
        ENHANCE_PARAMS["denoise_strength"],
        7,
        21,
    )
    gray = cv2.cvtColor(denoised, cv2.COLOR_BGR2GRAY)
    return cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        ENHANCE_PARAMS["adaptive_block"],
        11,
    )


def decode_qrcode(cv_image):
    """执行多尺度二维码解码"""
    processed = preprocess_image(cv_image)

    for scale in ENHANCE_PARAMS["scale_factors"]:
        scaled = cv2.resize(
            processed, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
        )

        decoded_objs = decode(scaled)
        if decoded_objs:
            return decoded_objs[0].data.decode("utf-8")

    return None


# ============ 文本识别部分 ============
# 文本识别字符集
alphabet = "0123456789abcdefghijklmnopqrstuvwxyz"

# 文本检测模型配置
TEXT_MODEL_PARAMS = {
    "model": "frozen_east_text_detection.pb",
    "ocr_model": "CRNN_VGG_BiLSTM_CTC.onnx",
    "width": 320,
    "height": 320,
    "thr": 0.5,
    "nms": 0.4,
}


def fourPointsTransform(frame, vertices):
    """透视变换裁剪文本区域"""
    vertices = np.asarray(vertices).astype(np.float32)
    outputSize = (100, 32)
    targetVertices = np.array(
        [
            [0, outputSize[1] - 1],
            [0, 0],
            [outputSize[0] - 1, 0],
            [outputSize[0] - 1, outputSize[1] - 1],
        ],
        dtype="float32",
    )

    rotationMatrix = cv2.getPerspectiveTransform(vertices, targetVertices)
    result = cv2.warpPerspective(frame, rotationMatrix, outputSize)
    return result


def decodeText(scores):
    """解码CRNN输出"""
    text = ""
    for i in range(scores.shape[0]):
        c = np.argmax(scores[i][0])
        if c != 0:
            text += alphabet[c - 1]
        else:
            text += "-"

    # 去除重复字符和分隔符
    char_list = []
    for i in range(len(text)):
        if text[i] != "-" and (not (i > 0 and text[i] == text[i - 1])):
            char_list.append(text[i])
    return "".join(char_list)


def decodeBoundingBoxes(scores, geometry, scoreThresh):
    """解码EAST输出"""
    detections = []
    confidences = []

    height = scores.shape[2]
    width = scores.shape[3]

    for y in range(height):
        scoresData = scores[0][0][y]
        x0_data = geometry[0][0][y]
        x1_data = geometry[0][1][y]
        x2_data = geometry[0][2][y]
        x3_data = geometry[0][3][y]
        anglesData = geometry[0][4][y]

        for x in range(width):
            score = scoresData[x]
            if score < scoreThresh:
                continue

            offsetX = x * 4.0
            offsetY = y * 4.0
            angle = anglesData[x]

            cosA = math.cos(angle)
            sinA = math.sin(angle)
            h = x0_data[x] + x2_data[x]
            w = x1_data[x] + x3_data[x]

            offset = (
                offsetX + cosA * x1_data[x] + sinA * x2_data[x],
                offsetY - sinA * x1_data[x] + cosA * x2_data[x],
            )

            p1 = (-sinA * h + offset[0], -cosA * h + offset[1])
            p3 = (-cosA * w + offset[0], sinA * w + offset[1])
            center = (0.5 * (p1[0] + p3[0]), 0.5 * (p1[1] + p3[1]))
            detections.append((center, (w, h), -angle * 180.0 / math.pi))
            confidences.append(float(score))

    return [detections, confidences]


def detect_text(cv_image):
    """文本检测和识别主函数"""
    # 加载模型
    detector = cv2.dnn.readNet(TEXT_MODEL_PARAMS["model"])
    recognizer = cv2.dnn.readNet(TEXT_MODEL_PARAMS["ocr_model"])

    # 首先将图像resize到800x600
    frame = cv2.resize(cv_image, (800, 600))
    height_resized, width_resized = frame.shape[:2]
    rW = width_resized / float(TEXT_MODEL_PARAMS["width"])
    rH = height_resized / float(TEXT_MODEL_PARAMS["height"])

    # 预处理（EAST模型输入）
    blob = cv2.dnn.blobFromImage(
        frame,
        1.0,
        (TEXT_MODEL_PARAMS["width"], TEXT_MODEL_PARAMS["height"]),
        (123.68, 116.78, 103.94),
        True,
        False,
    )

    # 文本检测
    detector.setInput(blob)
    outNames = ["feature_fusion/Conv_7/Sigmoid", "feature_fusion/concat_3"]
    scores, geometry = detector.forward(outNames)

    # 解码检测结果
    [boxes, confidences] = decodeBoundingBoxes(
        scores, geometry, TEXT_MODEL_PARAMS["thr"]
    )

    # 应用NMS
    indices = cv2.dnn.NMSBoxesRotated(
        boxes, confidences, TEXT_MODEL_PARAMS["thr"], TEXT_MODEL_PARAMS["nms"]
    )

    results = []

    if isinstance(indices, np.ndarray):
        indices = indices.flatten()

    for i in indices if indices is not None else []:
        # 获取旋转矩形顶点
        box = boxes[i]
        vertices = cv2.boxPoints(box)

        # 缩放回resize后的尺寸（800x600）
        vertices *= np.array([rW, rH])

        # 文本识别
        cropped = fourPointsTransform(frame, vertices)
        cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

        # CRNN输入预处理
        blob_ocr = cv2.dnn.blobFromImage(
            cropped,
            scalefactor=1 / 127.5,
            size=(100, 32),
            mean=127.5,
            swapRB=False,
            crop=False,
        )
        recognizer.setInput(blob_ocr)
        result = recognizer.forward()

        # 解码识别结果
        text = decodeText(result)
        results.append(text)

    return results


# ============ 主检测函数 ============
def text_detector(cv_image):
    """主检测函数：输入cv_image，输出识别结果；未识别到则返回空字符串"""

    if cv_image is None:
        return ""

    # 先尝试二维码识别
    qr_result = decode_qrcode(cv_image)
    if qr_result:
        qr_result = qr_result.lower()
        return qr_result[0] + qr_result[-1]

    # 再尝试文本识别
    text_results = detect_text(cv_image)

    for text in text_results:
        if "a" in text and "1" in text:
            return "a1"
        elif "a" in text and "2" in text:
            return "a2"
        elif "b" in text and "1" in text:
            return "b1"
        elif "b" in text and "2" in text:
            return "b2"

    # 如果都没有识别到
    return ""
