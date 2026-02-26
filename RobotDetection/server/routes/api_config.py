import logging
from flask import Blueprint, request, jsonify

from server.app import state, config
from vision.config import DetectorConfig

logger = logging.getLogger(__name__)
bp = Blueprint('api_config', __name__)


def _set_value(cfg, key_parts, value):
    """
    Walk dot-notation key parts into the config tree and set the value,
    casting to match the existing field's type.
    Returns True on success, False if key doesn't exist or cast fails.
    """
    obj = cfg
    for part in key_parts[:-1]:
        if not hasattr(obj, part):
            return False
        obj = getattr(obj, part)

    attr = key_parts[-1]
    if not hasattr(obj, attr):
        return False

    expected = type(getattr(obj, attr))
    try:
        if expected is bool:
            converted = bool(value)
        elif expected is int:
            converted = int(value)
        elif expected is float:
            converted = float(value)
        elif expected is str:
            converted = str(value)
        else:
            converted = value
        setattr(obj, attr, converted)
        return True
    except (ValueError, TypeError):
        return False


@bp.route("/api/config", methods=["GET"])
def get_config():
    """Full config snapshot. Used by the dashboard on load to seed all sliders."""
    with state.config_lock:
        return jsonify({
            "color":  {
                "red_factor":  config.color.red_factor,
                "blue_factor": config.color.blue_factor,
            },
            "bumper": {
                "min_area":   config.bumper.min_area,
                "min_aspect": config.bumper.min_aspect,
                "max_aspect": config.bumper.max_aspect,
            },
            "morph": {
                "kernel_size": config.morph.kernel_size,
                "iterations":  config.morph.iterations,
            },
            "metal": {
                "threshold":                config.metal.threshold,
                "search_height_multiplier": config.metal.search_height_multiplier,
                "dog_sigma1":               config.metal.dog_sigma1,
                "dog_sigma2":               config.metal.dog_sigma2,
                "dog_p":                    config.metal.dog_p,
                "dog_epsilon":              config.metal.dog_epsilon,
                "dog_phi":                  config.metal.dog_phi,
            },
            "debug": {
                "show_overlay": config.debug.show_overlay,
                "show_verbose": config.debug.show_verbose,
            },
            "performance": {
                "fps_smoothing_factor": config.performance.fps_smoothing_factor,
            },
            "camera": {
                "device": config.camera.device,
                "width":  config.camera.width,
                "height": config.camera.height,
            },
            "server": {
                "port": config.server.port,
                "host": config.server.host,
            },
        })


@bp.route("/api/config", methods=["POST"])
def update_config():
    """
    Update one or more config values. Body: {"color.red_factor": 2.5, ...}
    Acquires config_lock so detect() never reads a half-updated config.
    Saves to disk after any successful write.
    """
    data = request.json or {}
    updated, failed = {}, {}

    with state.config_lock:
        for key, value in data.items():
            if _set_value(config, key.split("."), value):
                updated[key] = value
            else:
                failed[key] = "invalid key or type"

        if updated:
            config.save()

    return jsonify({
        "status": "ok" if not failed else "partial",
        "updated": updated,
        "failed": failed,
    })


@bp.route("/api/config/save", methods=["POST"])
def save_config():
    """Force a manual save. Normally auto-saved after every POST."""
    with state.config_lock:
        ok = config.save()
    return jsonify({"status": "ok" if ok else "error"})


@bp.route("/api/config/reset", methods=["POST"])
def reset_config():
    """Reset all values to dataclass defaults and save."""
    with state.config_lock:
        config.from_dict(DetectorConfig().to_dict())
        config.save()
    return jsonify({"status": "ok", "message": "Reset to defaults"})
