import cv2
import time
import numpy as np
from typing import List, Tuple

from vision.detection_data import RobotDetection, DetectionResult
from vision.tracker import RobotTracker


def confirm_robots(
    img: np.ndarray,
    red_boxes: List[Tuple],
    blue_boxes: List[Tuple],
    metallic: np.ndarray,
    tracker: RobotTracker,
    frame_number: int,
    threshold: float,
    search_height_multiplier: float,
) -> Tuple[np.ndarray, DetectionResult]:
    """
    Filters bumper candidates by checking for a metallic robot body above each one,
    assigns persistent tracking IDs, and draws annotations onto the frame.

    For each bumper box (x, y, w, h):
        1. Define a search region above it: height = bumper_h * search_height_multiplier
        2. Compute what fraction of pixels in that region exceed the metallic threshold
        3. If fraction > 0.2, confirm as robot and create a RobotDetection

    All confirmed detections are passed to tracker.update() which assigns persistent
    IDs by nearest-neighbor matching against previous-frame positions.

    Returns:
        (annotated_frame, DetectionResult)
    """
    out = img.copy()
    detections = []

    threshold             = max(0.0, min(1.0, float(threshold)))
    search_height_multiplier = max(0.0, float(search_height_multiplier))

    def process(boxes, is_red):
        for x, y, w, h in boxes:
            search_h = int(h * search_height_multiplier)
            y0    = max(0, y - search_h)
            x_end = min(x + w, img.shape[1])

            if y0 >= y or x >= x_end or y >= img.shape[0]:
                continue

            region = metallic[y0:y, x:x_end]
            if region.size == 0:
                continue

            metallic_score = (region > threshold).mean()
            if metallic_score > 0.2:
                detections.append(RobotDetection(
                    x=x, y=y0,
                    width=w, height=y + h - y0,
                    center_x=x + w / 2.0,
                    center_y=(y0 + y + h) / 2.0,
                    confidence=min(1.0, metallic_score * 2.0),
                    is_red=is_red,
                    is_blue=not is_red,
                    timestamp=time.time()
                ))

    process(red_boxes,  is_red=True)
    process(blue_boxes, is_red=False)

    tracked = tracker.update(detections)

    for det in tracked:
        dot_color = (0, 0, 255) if det.is_red else (255, 0, 0)
        cv2.rectangle(out, (det.x, det.y), (det.x + det.width, det.y + det.height), (0, 255, 0), 3)
        cv2.putText(out, det.get_label(), (det.x, max(10, det.y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.circle(out, (int(det.center_x), int(det.center_y)), 5, dot_color, -1)

    return out, DetectionResult(robots=tracked, timestamp=time.time(), frame_number=frame_number)
