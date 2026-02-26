import psutil
import time
import threading


class SystemMonitor:
    """
    Polls CPU, memory, temperature, and disk usage in a background thread
    and exposes the latest readings via get_stats().

    WHY A BACKGROUND THREAD:
        psutil.cpu_percent(interval=0.1) blocks for 0.1 seconds to measure CPU.
        Calling this directly in a Flask route handler would add 100ms latency
        to every /api/stats request, which is polled every 100ms from the browser.
        Running it in a daemon thread means the measurements are always ready
        and get_stats() returns instantly.

    THREAD SAFETY:
        We don't use a lock here because reading/writing a float or None is
        atomic in CPython (GIL protects primitive assignments). A brief race
        where get_stats() reads a value mid-update is acceptable - it would
        just return a slightly stale reading, which is fine for a dashboard.

    STARTED BY:
        app.py at module level: system_monitor.start()
        daemon=True so it dies automatically when the main thread exits.
    """

    def __init__(self):
        self.cpu_percent = 0.0
        self.memory_percent = 0.0
        self.temperature = None  # None if temperature sensors unavailable
        self.disk_percent = 0.0
        self._running = False
        self._thread = None

    def start(self):
        """Spawn the background monitoring thread. Safe to call multiple times."""
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self._thread.start()

    def stop(self):
        """Signal the loop to stop and wait for the thread to exit (up to 1 second)."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _monitor_loop(self):
        """
        Background loop that updates stats every second.
        All stat reads are wrapped in try/except so a sensor failure on one
        metric doesn't prevent the others from updating.
        """
        while self._running:
            try:
                # cpu_percent(interval=0.1): measures CPU usage over a 0.1s window.
                # This is a blocking call - hence why it lives in its own thread.
                self.cpu_percent = psutil.cpu_percent(interval=0.1)
            except Exception:
                pass

            try:
                self.memory_percent = psutil.virtual_memory().percent
            except Exception:
                pass

            try:
                # sensors_temperatures() returns a dict of sensor_name -> [readings].
                # Sensor names vary by hardware. We try common ones in priority order,
                # falling back to whatever the first available sensor is.
                # On systems with no sensors (some VMs, some SBCs), this returns {}.
                temps = psutil.sensors_temperatures()
                self.temperature = None
                if temps:
                    for name in ['coretemp', 'cpu_thermal', 'k10temp', 'zenpower']:
                        if name in temps:
                            self.temperature = temps[name][0].current
                            break
                    if self.temperature is None:
                        # No known sensor found - use whatever is first available
                        first_sensor = list(temps.values())[0]
                        if first_sensor:
                            self.temperature = first_sensor[0].current
            except Exception:
                self.temperature = None

            try:
                self.disk_percent = psutil.disk_usage('/').percent
            except Exception:
                pass

            # Sleep 1 second between full update cycles.
            # System stats don't need sub-second precision for a dashboard.
            time.sleep(1.0)

    def get_stats(self):
        """
        Returns the latest cached stats as a plain dict.
        Called by /api/stats in app.py every 100ms.
        Returns instantly - all heavy work is done in _monitor_loop().
        temperature is None if sensors are unavailable; the browser checks for this
        and hides the temperature box in the UI.
        """
        return {
            'cpu_percent': round(self.cpu_percent, 1),
            'memory_percent': round(self.memory_percent, 1),
            'temperature': round(self.temperature, 1) if self.temperature else None,
            'disk_percent': round(self.disk_percent, 1)
        }
