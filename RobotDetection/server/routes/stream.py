import cv2
import time
import logging
from flask import Blueprint, Response, send_from_directory

from server.app import state, detector, nt_publisher

logger = logging.getLogger(__name__)
bp = Blueprint('stream', __name__)

MJPEG_MIME = "multipart/x-mixed-replace; boundary=frame"


def _encode(frame):
    """Encode a numpy frame to a MJPEG chunk. Returns bytes or None on failure."""
    ret, jpg = cv2.imencode(".jpg", frame)
    if not ret:
        return None
    return b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"


def _camera_frames(process: bool):
    """
    Generator that yields MJPEG chunks from the camera indefinitely.
    If process=True, runs the full detection pipeline and publishes to NT.
    """
    while True:
        with state.lock:
            frame = None if state.frame is None else state.frame.copy()

        if frame is None:
            time.sleep(0.01)
            continue

        try:
            if process:
                frame = detector.detect(frame)
                nt_publisher.publish(detector.latest_detections)

            chunk = _encode(frame)
            if chunk:
                yield chunk
            else:
                time.sleep(0.01)

        except Exception as e:
            logger.error(f"Stream error: {e}")
            time.sleep(0.01)


def _debug_frames(get_buf_fn, name: str):
    """Generator that yields MJPEG chunks from a detector debug buffer."""
    while True:
        buf = get_buf_fn()
        if buf is None:
            time.sleep(0.01)
            continue
        try:
            chunk = _encode(buf)
            if chunk:
                yield chunk
            else:
                time.sleep(0.01)
        except Exception as e:
            logger.error(f"{name} stream error: {e}")
            time.sleep(0.01)


# ── Video routes ──────────────────────────────────────────────────────────────

@bp.route("/raw")
def raw():
    """Raw camera feed with no processing."""
    return Response(_camera_frames(process=False), mimetype=MJPEG_MIME)


@bp.route("/processed")
def processed():
    """Detection-annotated camera feed."""
    return Response(_camera_frames(process=True), mimetype=MJPEG_MIME)


# ── Debug routes ──────────────────────────────────────────────────────────────

@bp.route("/debug/red")
def debug_red():
    """Red color mask. Tune color.red_factor until only bumpers are white."""
    return Response(_debug_frames(detector.get_red_mask, "red"), mimetype=MJPEG_MIME)


@bp.route("/debug/blue")
def debug_blue():
    """Blue color mask. Tune color.blue_factor."""
    return Response(_debug_frames(detector.get_blue_mask, "blue"), mimetype=MJPEG_MIME)


@bp.route("/debug/metallic")
def debug_metallic():
    """XDoG metallic map. Chassis should be white, walls/bumpers black."""
    return Response(_debug_frames(detector.get_metallic_buffer, "metallic"), mimetype=MJPEG_MIME)


# ── Dashboard ─────────────────────────────────────────────────────────────────

@bp.route("/")
def index():
    """Serve the dashboard HTML from server/static/."""
    response = send_from_directory('static', 'dashboard.html')
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response
