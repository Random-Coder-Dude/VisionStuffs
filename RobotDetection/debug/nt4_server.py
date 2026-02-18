"""
NT4 Server - Barebones
Just starts NT4 server on port 5810
"""

import ntcore
import time

print("🚀 Starting NT4 Server...")
inst = ntcore.NetworkTableInstance.getDefault()
inst.startServer()

print("✅ NT4 Server running on port 5810")
print("📡 Ready for connections")

# Track connection status
connected_clients = set()

def connection_listener(event):
    """This won't work - connections don't fire on topic listeners"""
    pass

# Connection events in pyntcore are tricky - they're logged but not easily hooked
# The client will show "CONNECTED" but server doesn't easily expose client list

# Workaround: Monitor a test topic to detect activity
vision = inst.getTable("Vision")

try:
    last_check = time.time()
    print("\n👀 Monitoring for client activity...\n")
    
    while True:
        # Check if we're getting data
        if time.time() - last_check > 2.0:
            has_targets = vision.getBoolean("hasTargets", None)
            if has_targets is not None:
                continue
            else:
                print(f"⏳ Waiting for clients...")
            last_check = time.time()
        
        time.sleep(0.5)
        
except KeyboardInterrupt:
    print("\n🛑 Shutting down...")
    inst.stopServer()
