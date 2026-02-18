import threading
import logging
import argparse
from server.camera import camera_loop
from server.app import app, state, config

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot Vision Detection Server')
    parser.add_argument('-height', type=int, help='Camera height')
    parser.add_argument('-width', type=int, help='Camera width')
    parser.add_argument('-deviceid', type=int, help='Camera device ID')
    parser.add_argument('-publishport', type=int, help='Server port')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    
    # Override config with CLI arguments if provided
    if args.deviceid is not None:
        config.camera.device = args.deviceid
    if args.width is not None:
        config.camera.width = args.width
    if args.height is not None:
        config.camera.height = args.height
    if args.publishport is not None:
        config.server.port = args.publishport
    
    # Use config values
    CAMERA_DEVICE = config.camera.device
    WIDTH = config.camera.width
    HEIGHT = config.camera.height
    PORT = config.server.port
    HOST = config.server.host
    
    # Validate configuration
    if WIDTH <= 0 or HEIGHT <= 0:
        logger.error(f"Invalid resolution: {WIDTH}x{HEIGHT}")
        exit(1)
    
    if PORT < 1 or PORT > 65535:
        logger.error(f"Invalid port: {PORT}")
        exit(1)
    
    logger.info(f"Starting camera thread with device {CAMERA_DEVICE} at {WIDTH}x{HEIGHT}")
    
    t = threading.Thread(
        target=camera_loop,
        args=(state, CAMERA_DEVICE, WIDTH, HEIGHT),
        daemon=True
    )
    t.start()

    logger.info(f"Starting Flask server on http://{HOST}:{PORT}")
    print(f"Running on http://{HOST}:{PORT}")
    
    try:
        app.run(host=HOST, port=PORT, threaded=True)
    except KeyboardInterrupt:
        logger.info("Shutting down gracefully...")
    except Exception as e:
        logger.error(f"Server error: {e}")
        exit(1)
