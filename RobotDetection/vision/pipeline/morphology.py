import cv2
import numpy as np


def build_kernel(kernel_size: int):
    """Builds an ellipse morphological kernel. Size is forced odd and clamped to [3, 31]."""
    k = int(kernel_size)
    if k % 2 == 0:
        k += 1
    k = max(3, min(k, 31))
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)), k


def clean_mask(mask: np.ndarray, kernel, iterations: int) -> np.ndarray:
    """
    Morphological CLOSE on a boolean mask to fill gaps inside color blobs.
    (e.g. black team numbers breaking up a red bumper region)

    Returns a uint8 array (0 or 255) ready for connectedComponentsWithStats().
    """
    iterations = max(1, int(iterations))
    return cv2.morphologyEx(
        (mask * 255).astype(np.uint8),
        cv2.MORPH_CLOSE,
        kernel,
        iterations=iterations
    )
