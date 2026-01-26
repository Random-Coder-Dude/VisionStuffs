import psutil
import time
import threading


class SystemMonitor:
    def __init__(self):
        self.cpu_percent = 0.0
        self.memory_percent = 0.0
        self.temperature = None
        self.disk_percent = 0.0
        self._running = False
        self._thread = None
        
    def start(self):
        """Start monitoring in background thread"""
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self._thread.start()
    
    def stop(self):
        """Stop monitoring"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
    
    def _monitor_loop(self):
        """Background monitoring loop"""
        while self._running:
            try:
                # CPU usage
                self.cpu_percent = psutil.cpu_percent(interval=0.1)
                
                # Memory usage
                mem = psutil.virtual_memory()
                self.memory_percent = mem.percent
                
                # Temperature (try to get CPU temp)
                try:
                    temps = psutil.sensors_temperatures()
                    if temps:
                        # Try common temperature sensor names
                        for name in ['coretemp', 'cpu_thermal', 'k10temp', 'zenpower']:
                            if name in temps:
                                self.temperature = temps[name][0].current
                                break
                        # If no known sensor, use first available
                        if self.temperature is None and temps:
                            first_sensor = list(temps.values())[0]
                            if first_sensor:
                                self.temperature = first_sensor[0].current
                except:
                    self.temperature = None
                
                # Disk usage
                disk = psutil.disk_usage('/')
                self.disk_percent = disk.percent
                
            except Exception as e:
                pass
            
            time.sleep(1.0)  # Update every second
    
    def get_stats(self):
        """Get current system stats"""
        return {
            'cpu_percent': round(self.cpu_percent, 1),
            'memory_percent': round(self.memory_percent, 1),
            'temperature': round(self.temperature, 1) if self.temperature else None,
            'disk_percent': round(self.disk_percent, 1)
        }
