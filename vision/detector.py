import cv2
import numpy as np
import time
import logging
from vision.metrics import RollingAverage

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BumperDetector:
    def __init__(self, config):
        self.cfg = config

        # Morph kernel (hot reload)
        self.kernel = None
        self._last_kernel_size = None
        
        # Initialize kernel immediately
        self._update_kernel_if_needed()

        self.fps_avg = RollingAverage(alpha=self.cfg.performance.fps_smoothing_factor)
        self._first_frame = True
        
        # Performance tracking
        self.timing_history = {
            "color_masks": [],
            "metallic": [],
            "morphology": [],
            "bbox": [],
            "robot": [],
            "total": [],
        }
        
        # Latest timings for display
        self.last_timings = {
            "color_masks": 0.0,
            "metallic": 0.0,
            "morphology": 0.0,
            "bbox": 0.0,
            "robot": 0.0,
            "total": 0.0,
        }

    # ---------- Morph kernel hot reload ----------
    def _update_kernel_if_needed(self):
        k = int(self.cfg.morph.kernel_size)
        # Ensure kernel size is odd
        if k % 2 == 0:
            k += 1
        # Clamp to reasonable range
        k = max(3, min(k, 31))
        
        if k != self._last_kernel_size:
            self.kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (k, k)
            )
            self._last_kernel_size = k

    # ---------- Color detection ----------
    def compute_color_masks(self, img):
        # Direct indexing - BGR format
        b = img[:, :, 0].astype(np.float32)
        g = img[:, :, 1].astype(np.float32)
        r = img[:, :, 2].astype(np.float32)

        # Validate factors
        red_factor = max(0.1, float(self.cfg.color.red_factor))
        blue_factor = max(0.1, float(self.cfg.color.blue_factor))

        # FIXED LOGIC:
        # Red dominance: red must be red_factor times GREATER than green and blue
        # This means: r >= g * red_factor AND r >= b * red_factor
        red_mask = (r >= g * red_factor) & (r >= b * red_factor)

        # Blue dominance: blue must be blue_factor times GREATER than red and green
        # This means: b >= r * blue_factor AND b >= g * blue_factor
        blue_mask = (b >= r * blue_factor) & (b >= g * blue_factor)

        return red_mask, blue_mask

    # ---------- Metallic detection ----------
    def compute_metallic(self, img):
        # Convert to float for precision
        b = img[:, :, 0].astype(np.float32)
        g = img[:, :, 1].astype(np.float32)
        r = img[:, :, 2].astype(np.float32)

        # Average of all channels
        avg = (r + g + b) / 3.0

        # Spread: how much channels deviate from average
        spread = (np.abs(r - avg) + np.abs(g - avg) + np.abs(b - avg)) / 3.0

        # Metallic score: high brightness, low spread (gray/silver)
        spread_weight = max(0.0, min(1.0, float(self.cfg.metal.spread_weight)))
        metallic = avg - spread_weight * spread
        
        # Normalize to 0-1 range
        return np.clip(metallic / 255.0, 0.0, 1.0)

    # ---------- Morph cleanup ----------
    def clean_mask(self, mask):
        if self.kernel is None:
            logger.warning("Kernel not initialized, skipping morphology")
            return (mask * 255).astype(np.uint8)
        
        iterations = max(1, int(self.cfg.morph.iterations))
        
        return cv2.morphologyEx(
            (mask * 255).astype(np.uint8),
            cv2.MORPH_CLOSE,
            self.kernel,
            iterations=iterations
        )

    # ---------- Bumper detection ----------
    def find_bumpers(self, mask):
        num, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        boxes = []

        min_area = max(1, int(self.cfg.bumper.min_area))
        min_aspect = max(0.1, float(self.cfg.bumper.min_aspect))
        max_aspect = max(min_aspect, float(self.cfg.bumper.max_aspect))

        for i in range(1, num):
            x, y, w, h, area = stats[i]
            
            # Skip if area is too small or dimensions are invalid
            if area < min_area or h <= 0 or w <= 0:
                continue

            aspect = w / h
            if min_aspect <= aspect <= max_aspect:
                boxes.append((x, y, w, h))

        return boxes

    # ---------- Robot confirmation ----------
    def confirm_robots(self, img, bboxes, metallic):
        out = img.copy()
        robot_count = 0

        threshold = max(0.0, min(1.0, float(self.cfg.metal.threshold)))
        search_mult = max(0.0, float(self.cfg.metal.search_height_multiplier))

        for x, y, w, h in bboxes:
            search_h = int(h * search_mult)
            y0 = max(0, y - search_h)

            # Ensure valid region bounds
            if y0 >= y or x >= img.shape[1] or y >= img.shape[0]:
                continue
            
            # Clamp x bounds
            x_end = min(x + w, img.shape[1])
            if x >= x_end:
                continue

            region = metallic[y0:y, x:x_end]
            if region.size == 0:
                continue

            # Check if region has enough metallic pixels
            if (region > threshold).mean() > 0.2:
                robot_count += 1
                cv2.rectangle(out, (x, y0), (x_end, y + h), (0, 255, 0), 2)
                cv2.putText(
                    out, "Robot", (x, max(10, y0 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

        return out, robot_count

    # ---------- Draw performance overlay ----------
    def _draw_overlay(self, img):
        y = 30
        
        # FPS
        fps_value = self.fps_avg.value if self.fps_avg.value is not None else 0.0
        cv2.putText(img, f"FPS: {fps_value:.1f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        y += 25
        total_ms = self.last_timings["total"] * 1000
        cv2.putText(img, f"Frame Time: {total_ms:.2f} ms", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Individual component times
        components = [
            ("Color", "color_masks"),
            ("Metallic", "metallic"),
            ("Morph", "morphology"),
            ("BBox", "bbox"),
            ("Robot", "robot")
        ]
        
        y += 20
        for label, key in components:
            y += 18
            ms = self.last_timings[key] * 1000
            cv2.putText(img, f"{label}: {ms:.2f} ms", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # ---------- Full pipeline ----------
    def detect(self, img):
        try:
            self._update_kernel_if_needed()
            t0 = time.perf_counter()

            # ---- Color masks ----
            t_color0 = time.perf_counter()
            red_mask, blue_mask = self.compute_color_masks(img)
            self.last_timings["color_masks"] = time.perf_counter() - t_color0

            # ---- Metallic ----
            t_metal0 = time.perf_counter()
            metallic = self.compute_metallic(img)
            self.last_timings["metallic"] = time.perf_counter() - t_metal0

            # ---- Morphology ----
            t_morph0 = time.perf_counter()
            red_clean = self.clean_mask(red_mask)
            blue_clean = self.clean_mask(blue_mask)
            self.last_timings["morphology"] = time.perf_counter() - t_morph0

            # ---- BBox ----
            t_bbox0 = time.perf_counter()
            red_boxes = self.find_bumpers(red_clean)
            blue_boxes = self.find_bumpers(blue_clean)
            all_boxes = red_boxes + blue_boxes
            self.last_timings["bbox"] = time.perf_counter() - t_bbox0

            # ---- Robot detection ----
            t_robot0 = time.perf_counter()
            out, robot_count = self.confirm_robots(img, all_boxes, metallic)
            self.last_timings["robot"] = time.perf_counter() - t_robot0

            # Total time
            total_time = time.perf_counter() - t0
            self.last_timings["total"] = total_time

            # Safe FPS calculation
            if total_time > 0.0001:  # Avoid division by very small numbers
                current_fps = 1.0 / total_time
                fps = self.fps_avg.update(current_fps)
            else:
                fps = self.fps_avg.value if not self._first_frame and self.fps_avg.value is not None else 0.0
            
            self._first_frame = False
            
            # Store timing history
            for key in self.timing_history:
                self.timing_history[key].append(self.last_timings[key])

            # Draw overlay
            if self.cfg.debug.show_overlay:
                self._draw_overlay(out)

            return out
            
        except Exception as e:
            logger.error(f"Detection error: {e}", exc_info=True)
            # Return original image on error
            return img.copy()
