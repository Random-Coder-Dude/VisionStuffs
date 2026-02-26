import cv2
import numpy as np
from typing import List, Tuple


def find_bumpers(mask: np.ndarray, min_area: int, min_aspect: float, max_aspect: float) -> List[Tuple]:
    """
    Finds connected blobs in a cleaned mask that are shaped like bumpers.

    Filters by:
        min_area:   removes noise / small reflections
        min_aspect: bumpers are wide (w/h >= min_aspect)
        max_aspect: rejects unrealistically long blobs

    Returns a list of (x, y, w, h) bounding boxes.
    """
    num, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)

    min_area   = max(1, int(min_area))
    min_aspect = max(0.1, float(min_aspect))
    max_aspect = max(min_aspect, float(max_aspect))

    boxes = []
    for i in range(1, num):  # skip component 0 = background
        x, y, w, h, area = stats[i]
        if area < min_area or w <= 0 or h <= 0:
            continue
        if min_aspect <= (w / h) <= max_aspect:
            boxes.append((x, y, w, h))

    return boxes
