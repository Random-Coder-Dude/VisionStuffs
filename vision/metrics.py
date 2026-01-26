class RollingAverage:
    def __init__(self, alpha=0.2):
        self.alpha = max(0.0, min(1.0, alpha))  # Clamp alpha to valid range
        self.value = None  # Use None to indicate uninitialized state

    def update(self, new_value):
        if self.value is None:
            # First update: use the new value directly
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
