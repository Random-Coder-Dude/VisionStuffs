class RollingAverage:
    """
    Exponential Moving Average (EMA) for smoothing noisy values like FPS.

    WHY NOT A SIMPLE AVERAGE:
        A simple average of the last N frames would require storing N values.
        An EMA achieves similar smoothing with just ONE stored value (self.value),
        by weighting recent samples more than old ones.

    HOW IT WORKS:
        new_value = alpha * raw_sample + (1 - alpha) * previous_average

        alpha = 0.2 (default):
            Each new sample contributes 20% to the result.
            The remaining 80% is the "memory" of all previous samples.
            Result: smooth, slow to react to sudden changes.

        alpha = 0.8:
            Each new sample contributes 80%.
            Result: responsive but jittery.

    USED BY:
        BumperDetector.fps_avg in detector.py.
        Updated once per frame with the raw instantaneous FPS (1.0 / frame_time).
        Read by /api/stats in app.py to populate the FPS chart in the dashboard.
    """

    def __init__(self, alpha=0.2):
        # Clamp alpha to [0, 1] - values outside this range make no mathematical sense.
        self.alpha = max(0.0, min(1.0, alpha))

        # None until the first update() call.
        # The first sample seeds self.value directly (no smoothing on frame 1),
        # which avoids the average starting at 0 and slowly climbing.
        self.value = None

    def update(self, new_value):
        if self.value is None:
            # First ever sample: use it as-is to seed the average.
            self.value = new_value
        else:
            # EMA formula: blend new sample with running average.
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value
