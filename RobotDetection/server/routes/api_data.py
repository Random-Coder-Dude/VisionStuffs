import time
import logging
from flask import Blueprint, jsonify

from server.app import detector, system_monitor

logger = logging.getLogger(__name__)
bp = Blueprint('api_data', __name__)


@bp.route("/api/stats")
def get_stats():
    """
    Performance + system stats. Polled every 100ms by the dashboard charts.
    """
    fps = detector.fps_avg.value or 0.0
    t = detector.last_timings

    return jsonify({
        "performance": {
            "fps": round(fps, 1),
            "frame_time_ms": round(t["total"] * 1000, 2),
            "timings": {
                "color_ms":    round(t["color_masks"] * 1000, 2),
                "metallic_ms": round(t["metallic"]    * 1000, 2),
                "morph_ms":    round(t["morphology"]  * 1000, 2),
                "bbox_ms":     round(t["bbox"]        * 1000, 2),
                "robot_ms":    round(t["robot"]       * 1000, 2),
            }
        },
        "system": system_monitor.get_stats()
    })


@bp.route("/api/detections")
def get_detections():
    """Latest detection results as JSON. Independently pollable by robot code."""
    try:
        if detector.latest_detections is None:
            return jsonify({
                "has_targets": False,
                "robot_count": 0,
                "red_count": 0,
                "blue_count": 0,
                "robots": [],
                "timestamp": time.time(),
                "frame_number": 0,
                "active_ids": {"red": [], "blue": []}
            })

        result = detector.latest_detections.to_dict()
        result["active_ids"] = detector.tracker.get_active_ids()
        return jsonify(result)

    except Exception as e:
        logger.error(f"get_detections error: {e}", exc_info=True)
        return jsonify({"error": str(e)}), 500


@bp.route("/api/tracking/reset", methods=["POST"])
def reset_tracking():
    """Wipe tracker state. All robots get fresh IDs next frame."""
    detector.tracker.reset()
    return jsonify({"status": "ok"})
