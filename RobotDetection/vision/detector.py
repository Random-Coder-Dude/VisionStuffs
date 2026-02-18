import cv2
import numpy as np
import time
import logging
from vision.metrics import RollingAverage
from vision.detection_data import RobotDetection, DetectionResult
from vision.tracker import RobotTracker


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class BumperDetector:
    def __init__(self, config):
        self.cfg = config

        # Morph kernel (hot reload)
        self.kernel = None
        self._last_kernel_size = None
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

        # DEBUG: Store intermediate results for visualization
        self.debug_red_mask = None
        self.debug_blue_mask = None
        self.debug_metallic = None

        # Tracking
        self.tracker = RobotTracker(max_distance=150.0, max_missing_frames=15)
        self.frame_number = 0
        self.latest_detections = None



    # ---------- Morph kernel hot reload ----------
    def _update_kernel_if_needed(self):
        k = int(self.cfg.morph.kernel_size)
        if k % 2 == 0:
            k += 1
        k = max(3, min(k, 31))
        if k != self._last_kernel_size:
            self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            self._last_kernel_size = k

    # ---------- Color detection ----------
    def compute_color_masks(self, img):
        b = img[:, :, 0].astype(np.float32)
        g = img[:, :, 1].astype(np.float32)
        r = img[:, :, 2].astype(np.float32)

        red_factor = max(0.1, float(self.cfg.color.red_factor))
        blue_factor = max(0.1, float(self.cfg.color.blue_factor))

        red_mask = (r >= g * red_factor) & (r >= b * red_factor)
        blue_mask = (b >= r * blue_factor) & (b >= g * blue_factor)

        return red_mask, blue_mask

    # ---------- Metallic detection ----------
    def compute_metallic(self, img):
        b = img[:, :, 0].astype(np.float32)
        g = img[:, :, 1].astype(np.float32)
        r = img[:, :, 2].astype(np.float32)

        avg = (r + g + b) / 3.0
        spread = (np.abs(r - avg) + np.abs(g - avg) + np.abs(b - avg)) / 3.0

        spread_weight = max(0.0, min(1.0, float(self.cfg.metal.spread_weight)))
        metallic = avg - spread_weight * spread

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
            if area < min_area or w <= 0 or h <= 0:
                continue
            aspect = w / h
            if min_aspect <= aspect <= max_aspect:
                boxes.append((x, y, w, h))
        return boxes

    # ---------- Robot confirmation ----------
    def confirm_robots(self, img, red_boxes, blue_boxes, metallic):
        """
        Confirm robots and return detection data
        Returns: (annotated_image, DetectionResult)
        """
        out = img.copy()
        detections = []

        threshold = max(0.0, min(1.0, float(self.cfg.metal.threshold)))
        search_mult = max(0.0, float(self.cfg.metal.search_height_multiplier))

        # Process red bumpers
        for x, y, w, h in red_boxes:
            search_h = int(h * search_mult)
            y0 = max(0, y - search_h)
            x_end = min(x + w, img.shape[1])
            if y0 >= y or x >= x_end or y >= img.shape[0]:
                continue

            region = metallic[y0:y, x:x_end]
            if region.size == 0:
                continue

            metallic_score = (region > threshold).mean()
            if metallic_score > 0.2:
                # Calculate center
                center_x = x + w / 2.0
                center_y = (y0 + y + h) / 2.0

                # Create detection
                detection = RobotDetection(
                    x=x,
                    y=y0,
                    width=w,
                    height=y + h - y0,
                    center_x=center_x,
                    center_y=center_y,
                    confidence=min(1.0, metallic_score * 2.0),
                    is_red=True,
                    is_blue=False,
                    timestamp=time.time()
                )

                detections.append(detection)

        # Process blue bumpers
        for x, y, w, h in blue_boxes:
            search_h = int(h * search_mult)
            y0 = max(0, y - search_h)
            x_end = min(x + w, img.shape[1])
            if y0 >= y or x >= x_end or y >= img.shape[0]:
                continue

            region = metallic[y0:y, x:x_end]
            if region.size == 0:
                continue

            metallic_score = (region > threshold).mean()
            if metallic_score > 0.2:
                center_x = x + w / 2.0
                center_y = (y0 + y + h) / 2.0

                detection = RobotDetection(
                    x=x,
                    y=y0,
                    width=w,
                    height=y + h - y0,
                    center_x=center_x,
                    center_y=center_y,
                    confidence=min(1.0, metallic_score * 2.0),
                    is_red=False,
                    is_blue=True,
                    timestamp=time.time()
                )

                detections.append(detection)

        # Apply tracking to assign IDs
        tracked_detections = self.tracker.update(detections)

        # Draw tracked robots on image
        for det in tracked_detections:
            color = (0, 0, 255) if det.is_red else (255, 0, 0)
            cv2.rectangle(out, (det.x, det.y), (det.x + det.width, det.y + det.height), (0, 255, 0), 3)

            # Draw label with ID
            label = det.get_label()
            cv2.putText(
                out, label, (det.x, max(10, det.y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )

            # Draw center point
            cx, cy = int(det.center_x), int(det.center_y)
            cv2.circle(out, (cx, cy), 5, color, -1)

        # Create result
        result = DetectionResult(
            robots=tracked_detections,
            timestamp=time.time(),
            frame_number=self.frame_number
        )

        return out, result

    # ---------- Draw raw bumper boxes ----------
    def _draw_raw_boxes(self, img, red_boxes, blue_boxes):
        out = img.copy()
        for x, y, w, h in red_boxes:
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
        for x, y, w, h in blue_boxes:
            cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 0), 2)
        return out

    # ---------- Draw performance overlay ----------
    def _draw_overlay(self, img):
        y = 30
        fps_value = self.fps_avg.value if self.fps_avg.value is not None else 0.0
        cv2.putText(img, f"FPS: {fps_value:.1f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        y += 25
        total_ms = self.last_timings["total"] * 1000
        cv2.putText(img, f"Frame Time: {total_ms:.2f} ms", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

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

    # ---------- Get debug images ----------
    def get_red_mask(self):
        """Get the red color mask as a displayable image"""
        if self.debug_red_mask is None:
            return None
        return self.debug_red_mask

    def get_blue_mask(self):
        """Get the blue color mask as a displayable image"""
        if self.debug_blue_mask is None:
            return None
        return self.debug_blue_mask

    def get_metallic_buffer(self):
        """Get the metallic detection buffer as a displayable image"""
        if self.debug_metallic is None:
            return None
        return self.debug_metallic

    # ---------- Full pipeline ----------
    def detect(self, img):
        try:
            self._update_kernel_if_needed()
            t0 = time.perf_counter()

            # ---- Color masks ----
            t_color0 = time.perf_counter()
            red_mask, blue_mask = self.compute_color_masks(img)
            # Store debug images (convert boolean masks to grayscale)
            self.debug_red_mask = (red_mask * 255).astype(np.uint8)
            self.debug_blue_mask = (blue_mask * 255).astype(np.uint8)
            self.last_timings["color_masks"] = time.perf_counter() - t_color0

            # ---- Metallic ----
            t_metal0 = time.perf_counter()
            metallic = self.compute_metallic(img)
            # Store debug image (normalize 0-1 to 0-255)
            self.debug_metallic = (metallic * 255).astype(np.uint8)
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

            # ---- Draw raw detections ----
            if getattr(self.cfg.debug, "show_raw_boxes", True):
                out = self._draw_raw_boxes(img, red_boxes, blue_boxes)
            else:
                out = img.copy()

            # ---- Robot detection ----
            t_robot0 = time.perf_counter()
            out, detection_result = self.confirm_robots(out, red_boxes, blue_boxes, metallic)
            self.latest_detections = detection_result
            self.last_timings["robot"] = time.perf_counter() - t_robot0

            # ---- Total time & FPS ----
            total_time = time.perf_counter() - t0
            self.last_timings["total"] = total_time

            if total_time > 0.0001:
                current_fps = 1.0 / total_time
                self.fps_avg.update(current_fps)
            fps_value = self.fps_avg.value if self.fps_avg.value is not None else 0.0
            self._first_frame = False

            # ---- Store timing history ----
            for key in self.timing_history:
                self.timing_history[key].append(self.last_timings[key])

            # ---- Draw overlay ----
            if getattr(self.cfg.debug, "show_overlay", True):
                self._draw_overlay(out)

            self.frame_number += 1
            return out

        except Exception as e:
            logger.error(f"Detection error: {e}", exc_info=True)
            return img.copy()
