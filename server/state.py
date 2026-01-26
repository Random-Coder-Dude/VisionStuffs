import threading

class SharedState:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()
