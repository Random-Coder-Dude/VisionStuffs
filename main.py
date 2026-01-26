import threading
from server.camera import camera_loop
from server.app import app, state

CAMERA_DEVICE = 0
WIDTH = 640
HEIGHT = 480
PORT = 1403

if __name__ == "__main__":
    t = threading.Thread(
        target=camera_loop,
        args=(state, CAMERA_DEVICE, WIDTH, HEIGHT),
        daemon=True
    )
    t.start()

    print(f"Running on http://0.0.0.0:{PORT}")
    app.run(host="0.0.0.0", port=PORT, threaded=True)
