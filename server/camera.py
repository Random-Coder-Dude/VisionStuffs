import cv2
import time

def camera_loop(state, device, width, height):
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    while True:
        ret, frame = cap.read()
        if ret:
            with state.lock:
                state.frame = frame
        time.sleep(0.01)
