import cv2
import numpy as np
import math
import time


# ----------------- 偏航检测 -----------------
def safe_fitline(points):
    """安全地处理 fitLine 返回类型"""
    line_params = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    if isinstance(line_params, list) and len(line_params) == 4:
        vx = (
            line_params[0][0] if hasattr(line_params[0], "__iter__") else line_params[0]
        )
        vy = (
            line_params[1][0] if hasattr(line_params[1], "__iter__") else line_params[1]
        )
        x0 = (
            line_params[2][0] if hasattr(line_params[2], "__iter__") else line_params[2]
        )
        y0 = (
            line_params[3][0] if hasattr(line_params[3], "__iter__") else line_params[3]
        )
        return vx, vy, x0, y0
    return line_params


def detect_deviation(cv_image, S_road=False):
    """检测图像的偏离角度，返回连续偏航角度（正：右偏，负：左偏）"""
    h, w = cv_image.shape[:2]
    max_dimension = 800
    scale = max_dimension / max(h, w)
    nw, nh = int(w * scale), int(h * scale)
    img = cv2.resize(cv_image, (nw, nh), interpolation=cv2.INTER_AREA)

    mask = np.zeros((nh, nw), dtype=np.uint8)
    if S_road:
        bw = nw * 0.60
        tw = nw * 0.40
        hr = 0.10
    else:
        bw = nw * 0.40
        tw = nw * 0.25
        hr = 0.12
    pts = np.array(
        [
            [(nw - bw) // 2, nh],
            [(nw + bw) // 2, nh],
            [(nw + tw) // 2, int(nh * (1 - hr))],
            [(nw - tw) // 2, int(nh * (1 - hr))],
        ],
        np.int32,
    )
    cv2.fillPoly(mask, [pts], 255)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([40, 255, 255]))
    masked = cv2.bitwise_and(yellow_mask, yellow_mask, mask=mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    proc = cv2.morphologyEx(masked, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(proc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in cnts if cv2.contourArea(c) > 20]

    deviation_angle = 0.0
    if valid:
        pts_all = np.vstack([c.squeeze() for c in valid])
        miny, maxy = pts_all[:, 1].min(), pts_all[:, 1].max()
        centers = []
        for y in range(int(miny), int(maxy), 5):
            row = pts_all[np.abs(pts_all[:, 1] - y) < 0.5]
            if len(row) > 1:
                xs = sorted(row[:, 0])
                centers.append([(xs[0] + xs[-1]) / 2, y])
        cp = np.array(centers)
        if cp.size:
            bc = tuple(cp[cp[:, 1].argmax()])
            start_y = max(miny, bc[1] - 30)
            bp = cp[cp[:, 1] >= start_y]
            if len(bp) > 1:
                vx, vy, x0, y0 = map(float, safe_fitline(bp))
                ang = math.degrees(math.atan2(vy, vx) - math.pi / 2)
                if ang > 90:
                    ang -= 180
                if ang < -90:
                    ang += 180
                deviation_angle = ang

    return deviation_angle  # 连续偏航角，单位：度
