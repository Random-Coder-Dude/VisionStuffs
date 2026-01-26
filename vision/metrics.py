class RollingAverage:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.value = 0.0

    def update(self, new_value):
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
