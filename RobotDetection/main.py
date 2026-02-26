import threading
import logging
import argparse
from server.camera import camera_loop
from server.app import app, state, config

# ============================================================
# LOGGING SETUP
# ============================================================
# basicConfig sets the format and minimum level for all loggers in the process.
# INFO means we see INFO, WARNING, ERROR, CRITICAL but not DEBUG.
# Each module creates its own logger (e.g., logger = logging.getLogger(__name__))
# and they all inherit this config.
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def parse_args():
    """
    Defines command-line arguments so you can override config without editing files.
    
    Example usage:
        python main.py -deviceid 1 -width 1280 -height 720 -publishport 5800

    Any argument not provided stays as whatever value is in vision_config.json
    (or the dataclass default if no config file exists).
    argparse automatically generates --help output from the description/help strings.
    """
    parser = argparse.ArgumentParser(description='Robot Vision Detection Server')
    parser.add_argument('-height',      type=int, help='Camera capture height in pixels')
    parser.add_argument('-width',       type=int, help='Camera capture width in pixels')
    parser.add_argument('-deviceid',    type=int, help='V4L2 camera device index (0=/dev/video0)')
    parser.add_argument('-publishport', type=int, help='Port for the Flask web server')
    return parser.parse_args()


if __name__ == "__main__":
    # ============================================================
    # STARTUP SEQUENCE
    # ============================================================
    # Order matters here:
    #   1. Parse args
    #   2. Apply args on top of config (config was already loaded from disk in app.py)
    #   3. Validate
    #   4. Start camera thread
    #   5. Start Flask (blocks until Ctrl+C or crash)

    args = parse_args()

    # Apply command-line overrides. We only override values that were explicitly passed.
    # "is not None" check is how argparse signals "this arg was provided."
    # This lets you override just one thing (e.g., -deviceid 1) without specifying all args.
    if args.deviceid is not None:
        config.camera.device = args.deviceid
    if args.width is not None:
        config.camera.width = args.width
    if args.height is not None:
        config.camera.height = args.height
    if args.publishport is not None:
        config.server.port = args.publishport

    # Snapshot final values into local variables for clarity in logs + thread args.
    # (config could theoretically change later via /api/config, but camera and server
    # settings are only read at startup so we capture them now.)
    CAMERA_DEVICE = config.camera.device
    WIDTH         = config.camera.width
    HEIGHT        = config.camera.height
    PORT          = config.server.port
    HOST          = config.server.host

    # ---- Validation ----
    # Fail fast with a clear error rather than letting OpenCV or Flask crash with a
    # cryptic error message. exit(1) signals a non-zero exit code to the shell/systemd.
    if WIDTH <= 0 or HEIGHT <= 0:
        logger.error(f"Invalid resolution: {WIDTH}x{HEIGHT}")
        exit(1)

    if PORT < 1 or PORT > 65535:
        logger.error(f"Invalid port: {PORT}")
        exit(1)

    # ---- Start camera thread ----
    # camera_loop() runs forever in a background thread, capturing frames and
    # writing them to state.frame under state.lock.
    # Flask's stream() generator in app.py reads state.frame to serve MJPEG video.
    #
    # daemon=True: thread dies automatically when the main thread (Flask) exits.
    # This means Ctrl+C on the Flask server cleanly kills the camera thread too.
    logger.info(f"Starting camera thread with device {CAMERA_DEVICE} at {WIDTH}x{HEIGHT}")
    camera_thread = threading.Thread(
        target=camera_loop,
        args=(state, CAMERA_DEVICE, WIDTH, HEIGHT),
        daemon=True
    )
    camera_thread.start()

    # ---- Start Flask web server ----
    # This call BLOCKS until the server is stopped (Ctrl+C or unhandled exception).
    # Everything after this line only runs on shutdown.
    #
    # host=HOST ("0.0.0.0"): accept connections on all network interfaces.
    #   Necessary so other computers on the network can reach the dashboard.
    #   If you only want localhost access, change to "127.0.0.1" in config.
    #
    # threaded=True: Flask handles each HTTP request in its own thread.
    #   Without this, a slow /processed MJPEG stream would block all other requests.
    #   With it, /api/stats polling and slider updates work concurrently with streaming.
    logger.info(f"Starting Flask server on http://{HOST}:{PORT}")
    print(f"Dashboard: http://{HOST}:{PORT}")

    try:
        app.run(host=HOST, port=PORT, threaded=True)
    except KeyboardInterrupt:
        # Ctrl+C: clean intentional shutdown
        logger.info("Shutting down gracefully...")
    except Exception as e:
        # Unexpected crash: log and exit with error code
        logger.error(f"Server error: {e}")
        exit(1)
