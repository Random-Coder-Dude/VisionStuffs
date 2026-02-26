import numpy as np


def compute_color_masks(img, red_factor: float, blue_factor: float):
    """
    Returns boolean arrays (red_mask, blue_mask) of which pixels pass the color ratio test.
    img is a BGR uint8 frame. Factors come from ColorConfig.
    """
    b = img[:, :, 0].astype(np.float32)
    g = img[:, :, 1].astype(np.float32)
    r = img[:, :, 2].astype(np.float32)

    red_factor  = max(0.1, float(red_factor))
    blue_factor = max(0.1, float(blue_factor))

    red_mask  = (r >= g * red_factor)  & (r >= b * red_factor)
    blue_mask = (b >= r * blue_factor) & (b >= g * blue_factor)

    return red_mask, blue_mask
