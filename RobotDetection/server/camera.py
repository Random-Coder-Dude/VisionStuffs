import cv2
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def camera_loop(state, device, width, height):
    cap = None
    retry_delay = 1.0
    
    while True:
        try:
            if cap is None:
                logger.info(f"Attempting to open camera device {device}")
                # Use DirectShow on Windows for faster initialization
                cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
                
                if not cap.isOpened():
                    logger.error(f"Failed to open camera device {device}")
                    cap = None
                    time.sleep(retry_delay)
                    retry_delay = min(retry_delay * 2, 10.0)  # Exponential backoff
                    continue
                
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                logger.info(f"Camera opened successfully: {width}x{height}")
                retry_delay = 1.0  # Reset retry delay
            
            ret, frame = cap.read()
            
            if not ret:
                logger.warning("Failed to read frame from camera, reconnecting...")
                cap.release()
                cap = None
                time.sleep(retry_delay)
                continue
            
            with state.lock:
                state.frame = frame
            
            time.sleep(0.01)
            
        except Exception as e:
            logger.error(f"Camera loop error: {e}")
            if cap is not None:
                try:
                    cap.release()
                except:
                    pass
                cap = None
            time.sleep(retry_delay)
