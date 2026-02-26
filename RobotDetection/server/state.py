import threading


class SharedState:
    """
    A shared memory container that safely bridges the camera thread and the Flask web server.

    THE PROBLEM THIS SOLVES:
        The system has two threads running simultaneously:
            1. camera_loop() in camera.py  -> runs in a background daemon thread
                                             -> constantly WRITES new frames to state.frame
            2. Flask routes in app.py       -> runs on the main thread (+ per-request threads)
                                             -> constantly READS state.frame to stream to browser

        If both threads touch state.frame at the same time without coordination, you get a
        "race condition" - Flask might read a frame while the camera is halfway through
        writing it, resulting in a corrupted half-old/half-new frame being sent to the browser.

    THE SOLUTION - threading.Lock():
        A Lock is a mutual exclusion primitive. Only ONE thread can hold it at a time.
        Any thread that tries to acquire it while another holds it will BLOCK (pause) until
        the holder releases it. We use Python's "with" statement to auto-release on exit.

        Usage pattern:
            # Writer (camera thread):
            with state.lock:
                state.frame = new_frame   # lock held, nobody else can read mid-write

            # Reader (flask thread):
            with state.lock:
                frame = state.frame.copy()  # lock held, camera can't overwrite mid-copy

    WHY TWO SEPARATE LOCKS:
        We use state.lock for frames and state.config_lock for config.
        If we used ONE lock for everything, a config update from the UI slider would
        block the video stream (and vice versa). Separating them means:
            - config updates don't stall video
            - video streaming doesn't block config saves
    """

    def __init__(self):
        # The most recent raw frame captured from the camera.
        # Written by camera_loop() in camera.py.
        # Read (and .copy()'d) by the stream() generator in app.py.
        # Starts as None - stream() checks for None and skips until camera is ready.
        self.frame = None

        # Protects state.frame against simultaneous read/write from camera + Flask threads.
        self.lock = threading.Lock()

        # Protects the DetectorConfig object (vision/config.py).
        # The /api/config POST route acquires this before changing any slider value,
        # ensuring the BumperDetector never reads a half-updated config mid-frame.
        self.config_lock = threading.Lock()
