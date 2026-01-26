import threading

class SharedState:
    def __init__(self):
        self.frame = None
        self.lock = threading.Lock()
        self.config_lock = threading.Lock()  # Separate lock for config updates
