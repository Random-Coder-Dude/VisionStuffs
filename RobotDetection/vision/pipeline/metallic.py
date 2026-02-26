import cv2
import numpy as np


def _sigma_to_ksize(sigma: float) -> int:
    """Converts a Gaussian sigma to a valid odd kernel size."""
    k = int(6 * sigma + 1)
    return k if k % 2 == 1 else k + 1


def compute_xdog(img, sigma1: float, sigma2: float, p: float, epsilon: float, phi: float) -> np.ndarray:
    """
    XDoG (Extended Difference of Gaussians) metallic/texture detector.

    Implements Winnemoeller et al. 2012 (Eq. 7 + Eq. 5):
        S(x) = (1 + p) * G_sigma1 - p * G_sigma2     [sharpening]
        T(u) = 1                    if u >= epsilon
               1 + tanh(phi*(u-e))  otherwise          [soft threshold]

    Args:
        img:     BGR uint8 frame
        sigma1:  fine Gaussian sigma
        sigma2:  coarse Gaussian sigma (must be > sigma1)
        p:       edge emphasis strength (~20 per paper)
        epsilon: threshold level in [0,1]
        phi:     transition sharpness (high = near-hard threshold)

    Returns:
        float32 HxW array [0, 1] where bright = metallic/textured
    """
    sigma1  = max(0.1, float(sigma1))
    sigma2  = max(sigma1 + 0.1, float(sigma2))
    p       = max(0.0, float(p))
    phi     = max(0.1, float(phi))
    epsilon = float(epsilon)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0

    blur1 = cv2.GaussianBlur(gray, (_sigma_to_ksize(sigma1),) * 2, sigma1)
    blur2 = cv2.GaussianBlur(gray, (_sigma_to_ksize(sigma2),) * 2, sigma2)

    xdog = (1.0 + p) * blur1 - p * blur2

    result = np.where(
        xdog >= epsilon,
        np.ones_like(xdog),
        1.0 + np.tanh(phi * (xdog - epsilon))
    )

    return np.clip(result, 0.0, 1.0).astype(np.float32)
