import cv2
import numpy as np
import time
from vision.metrics import RollingAverage


class BumperDetector:
    def __init__(self, config):
        self.cfg = config

        # Morph kernel (hot reload)
        self.kernel = None
        self._last_kernel_size = None

        self.fps_avg = RollingAverage()

    # ---------- Morph kernel hot reload ----------
    def _update_kernel_if_needed(self):
        k = int(self.cfg.morph.kernel_size)
        if k != self._last_kernel_size:
            self.kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (k, k)
            )
            self._last_kernel_size = k

    # ---------- Color detection ----------
    def compute_color_masks(self, img):
        b, g, r = cv2.split(img)

        red = (r >= g * self.cfg.color.red_factor) & \
              (r >= b * self.cfg.color.red_factor)

        blue = (b >= g * self.cfg.color.blue_factor) & \
               (b >= r * self.cfg.color.blue_factor)

        return red, blue

    # ---------- Metallic detection ----------
    def compute_metallic(self, img):
        img_f = img.astype(np.float32)

        avg = img_f.mean(axis=2)
        spread = np.abs(img_f - avg[..., None]).mean(axis=2)

        metallic = avg - self.cfg.metal.spread_weight * spread
        return np.clip(metallic / 255.0, 0.0, 1.0)

    # ---------- Morph cleanup ----------
    def clean_mask(self, mask):
        return cv2.morphologyEx(
            (mask * 255).astype(np.uint8),
            cv2.MORPH_CLOSE,
            self.kernel,
            iterations=int(self.cfg.morph.iterations)
        )

    # ---------- Bumper detection ----------
    def find_bumpers(self, mask):
        num, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        boxes = []

        for i in range(1, num):
            x, y, w, h, area = stats[i]
            if area < self.cfg.bumper.min_area or h == 0:
                continue

            aspect = w / h
            if self.cfg.bumper.min_aspect <= aspect <= self.cfg.bumper.max_aspect:
                boxes.append((x, y, w, h))

        return boxes

    # ---------- Robot confirmation ----------
    def confirm_robots(self, img, bboxes, metallic):
        out = img.copy()

        for x, y, w, h in bboxes:
            search_h = int(h * self.cfg.metal.search_height_multiplier)
            y0 = max(0, y - search_h)

            region = metallic[y0:y, x:x + w]
            if region.size == 0:
                continue

            if (region > self.cfg.metal.threshold).mean() > 0.2:
                cv2.rectangle(out, (x, y0), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    out, "Robot", (x, y0 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

        return out

    # ---------- Full pipeline ----------
    def detect(self, img):
        self._update_kernel_if_needed()
        t0 = time.perf_counter()

        red, blue = self.compute_color_masks(img)
        metallic = self.compute_metallic(img)

        red = self.clean_mask(red)
        blue = self.clean_mask(blue)

        bboxes = self.find_bumpers(red) + self.find_bumpers(blue)
        out = self.confirm_robots(img, bboxes, metallic)

        fps = self.fps_avg.update(1.0 / (time.perf_counter() - t0))

        if self.cfg.debug.show_overlay:
            cv2.putText(
                out, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )

        return out
