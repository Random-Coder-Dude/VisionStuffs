import cv2
import time
import logging

logger = logging.getLogger(__name__)


def camera_loop(state, device, width, height):
    """
    Runs forever in a background daemon thread started by main.py.
    Its only job: capture frames from the camera and write them to state.frame
    so the Flask stream() generator in app.py can serve them to the browser.

    THREAD RELATIONSHIP:
        main.py starts this function in a daemon thread:
            t = threading.Thread(target=camera_loop, args=(state, ...), daemon=True)
            t.start()
        "daemon=True" means this thread automatically dies when the main thread exits,
        so we don't need an explicit shutdown signal.

        state is a SharedState instance (server/state.py). It holds state.frame and
        state.lock. We acquire the lock before writing to prevent Flask from reading
        a half-written frame on another thread.

    RECONNECTION LOGIC:
        cap starts as None. Whenever the camera fails (open fails or read fails),
        cap is set back to None, which causes the next loop iteration to re-open it.
        This means the camera self-heals from:
            - USB disconnect/reconnect
            - Camera not ready at startup
            - Driver crashes
    """
    cap = None

    # retry_delay controls how long we wait between failed open attempts.
    # It grows exponentially (1s, 2s, 4s, 8s, 10s max) so we don't spam
    # failed open() calls when the camera is genuinely unavailable.
    retry_delay = 1.0

    while True:
        try:
            # ---- Camera initialization ----
            # Only runs when cap is None (first start or after a failure)
            if cap is None:
                logger.info(f"Attempting to open camera device {device}")

                # cv2.CAP_V4L2 = Video4Linux2, the native Linux camera backend.
                # More reliable than letting OpenCV auto-detect on Linux.
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

                if not cap.isOpened():
                    logger.error(f"Failed to open camera device {device}")
                    cap = None
                    time.sleep(retry_delay)
                    # Exponential backoff: double the wait each failure, cap at 10s.
                    # Prevents hammering a camera that's not plugged in.
                    retry_delay = min(retry_delay * 2, 10.0)
                    continue  # Jump back to top of while loop to retry

                # Tell the driver what resolution we want.
                # Note: the driver may silently round to the nearest supported resolution.
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                logger.info(f"Camera opened successfully: {width}x{height}")

                # Reset backoff so next failure starts fresh from 1s
                retry_delay = 1.0

            # ---- Frame capture ----
            # cap.read() blocks until a new frame is available from the driver,
            # then returns (success_bool, frame_array).
            # This is why we don't need an explicit sleep for frame rate pacing here -
            # the camera hardware controls the pace (usually 30fps).
            ret, frame = cap.read()

            if not ret:
                # Camera returned a bad frame - could be disconnect, driver error, etc.
                # Release and set cap=None so next iteration tries to reconnect.
                logger.warning("Failed to read frame from camera, reconnecting...")
                cap.release()
                cap = None
                time.sleep(retry_delay)
                continue

            # ---- Write frame to shared state ----
            # Acquire the lock before writing so the Flask stream() thread
            # on app.py can't read state.frame while we're in the middle of replacing it.
            # "with state.lock:" automatically releases the lock when the block exits,
            # even if an exception is raised.
            with state.lock:
                state.frame = frame

            # Small sleep to cap CPU usage.
            # cap.read() already blocks on the camera's frame rate, so this mostly
            # acts as a safety valve in case the driver returns immediately for some reason.
            time.sleep(0.01)

        except Exception as e:
            # Catch-all so no error can kill this thread.
            # After any unexpected crash, release the camera and retry from scratch.
            logger.error(f"Camera loop error: {e}")
            if cap is not None:
                try:
                    cap.release()
                except:
                    pass  # release() itself can fail if the driver is in a bad state
                cap = None
            time.sleep(retry_delay)
