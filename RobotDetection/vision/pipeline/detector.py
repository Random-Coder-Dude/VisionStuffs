import cv2
import numpy as np
import time
import logging

from vision.config import DetectorConfig
from vision.metrics import RollingAverage
from vision.detection_data import DetectionResult
from vision.tracker import RobotTracker
from vision.pipeline.color import compute_color_masks
from vision.pipeline.metallic import compute_xdog
from vision.pipeline.morphology import build_kernel, clean_mask
from vision.pipeline.bumper import find_bumpers
from vision.pipeline.confirm import confirm_robots

logger = logging.getLogger(__name__)


class BumperDetector:
    """
    Orchestrates the 5-stage vision pipeline. Each stage lives in its own module:

        vision/pipeline/color.py      → compute_color_masks()
        vision/pipeline/metallic.py   → compute_xdog()
        vision/pipeline/morphology.py → clean_mask()
        vision/pipeline/bumper.py     → find_bumpers()
        vision/pipeline/confirm.py    → confirm_robots()

    This class owns shared state that must persist across frames:
        - morphological kernel (rebuilt when kernel_size config changes)
        - RobotTracker (maintains track IDs across frames)
        - timing measurements
        - debug buffers (intermediate images served by /debug/* routes)
        - latest_detections (read by /api/detections and nt_publisher)

    Config hot-reload:
        self.cfg is the shared DetectorConfig from server/app.py.
        detect() reads it every frame so slider changes from /api/config
        take effect on the very next frame without restarting.
    """

    def __init__(self, config: DetectorConfig):
        self.cfg = config

        self.kernel, self._last_kernel_size = build_kernel(self.cfg.morph.kernel_size)
        self.fps_avg = RollingAverage(alpha=self.cfg.performance.fps_smoothing_factor)

        self.last_timings = {
            "color_masks": 0.0,
            "metallic":    0.0,
            "morphology":  0.0,
            "bbox":        0.0,
            "robot":       0.0,
            "total":       0.0,
        }

        self.debug_red_mask  = None
        self.debug_blue_mask = None
        self.debug_metallic  = None

        self.tracker           = RobotTracker(max_distance=150.0, max_missing_frames=15)
        self.frame_number      = 0
        self.latest_detections = None

    # ── Kernel hot-reload ─────────────────────────────────────────────────────

    def _refresh_kernel(self):
        if int(self.cfg.morph.kernel_size) != self._last_kernel_size:
            self.kernel, self._last_kernel_size = build_kernel(self.cfg.morph.kernel_size)

    # ── Debug buffer accessors (called by /debug/* routes in stream.py) ───────

    def get_red_mask(self):
        return self.debug_red_mask

    def get_blue_mask(self):
        return self.debug_blue_mask

    def get_metallic_buffer(self):
        return self.debug_metallic

    # ── Optional overlay ──────────────────────────────────────────────────────

    def _draw_overlay(self, img):
        fps = self.fps_avg.value or 0.0
        cv2.putText(img, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(img, f"Frame: {self.last_timings['total']*1000:.2f}ms", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        labels = [("Color", "color_masks"), ("XDoG", "metallic"),
                  ("Morph", "morphology"),  ("BBox", "bbox"), ("Robot", "robot")]
        y = 75
        for label, key in labels:
            y += 18
            cv2.putText(img, f"{label}: {self.last_timings[key]*1000:.2f}ms", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # ── Main entry point ──────────────────────────────────────────────────────

    def detect(self, img: np.ndarray) -> np.ndarray:
        """
        Run the full pipeline on one frame. Called once per frame by stream.py.
        Returns an annotated copy of the frame.
        On any exception, logs and returns an unannotated copy so the stream stays alive.
        """
        try:
            self._refresh_kernel()
            t0 = time.perf_counter()

            # 1. Color masks
            t = time.perf_counter()
            red_mask, blue_mask = compute_color_masks(
                img, self.cfg.color.red_factor, self.cfg.color.blue_factor)
            self.debug_red_mask  = (red_mask  * 255).astype(np.uint8)
            self.debug_blue_mask = (blue_mask * 255).astype(np.uint8)
            self.last_timings["color_masks"] = time.perf_counter() - t

            # 2. XDoG metallic map
            t = time.perf_counter()
            metallic = compute_xdog(
                img,
                sigma1=self.cfg.metal.dog_sigma1,
                sigma2=self.cfg.metal.dog_sigma2,
                p=self.cfg.metal.dog_p,
                epsilon=self.cfg.metal.dog_epsilon,
                phi=self.cfg.metal.dog_phi,
            )
            self.debug_metallic = (metallic * 255).astype(np.uint8)
            self.last_timings["metallic"] = time.perf_counter() - t

            # 3. Morphological cleanup
            t = time.perf_counter()
            red_clean  = clean_mask(red_mask,  self.kernel, self.cfg.morph.iterations)
            blue_clean = clean_mask(blue_mask, self.kernel, self.cfg.morph.iterations)
            self.last_timings["morphology"] = time.perf_counter() - t

            # 4. Find bumper-shaped blobs
            t = time.perf_counter()
            red_boxes  = find_bumpers(red_clean,  self.cfg.bumper.min_area,
                                      self.cfg.bumper.min_aspect, self.cfg.bumper.max_aspect)
            blue_boxes = find_bumpers(blue_clean, self.cfg.bumper.min_area,
                                      self.cfg.bumper.min_aspect, self.cfg.bumper.max_aspect)
            self.last_timings["bbox"] = time.perf_counter() - t

            # 5. Confirm via metallic check + track + annotate
            t = time.perf_counter()
            out, result = confirm_robots(
                img, red_boxes, blue_boxes, metallic,
                tracker=self.tracker,
                frame_number=self.frame_number,
                threshold=self.cfg.metal.threshold,
                search_height_multiplier=self.cfg.metal.search_height_multiplier,
            )
            self.latest_detections = result
            self.last_timings["robot"] = time.perf_counter() - t

            total = time.perf_counter() - t0
            self.last_timings["total"] = total
            if total > 0.0001:
                self.fps_avg.update(1.0 / total)

            if getattr(self.cfg.debug, "show_overlay", False):
                self._draw_overlay(out)

            self.frame_number += 1
            return out

        except Exception as e:
            logger.error(f"Detection error: {e}", exc_info=True)
            return img.copy()
