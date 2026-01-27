"""
NetworkTables CLIENT test - connects to Orange Pi as server
Run this on Windows to verify NT data from Orange Pi
"""
from networktables import NetworkTables
import time

# Connect to Orange Pi (which is running as NT server)
ORANGE_PI_IP = "10.0.0.207"

print(f"🔌 Connecting to NT Server at {ORANGE_PI_IP}...")
NetworkTables.initialize(server=ORANGE_PI_IP)

print("📡 Waiting for connection...")
while not NetworkTables.isConnected():
    time.sleep(0.1)

print("✅ Connected!")

vision = NetworkTables.getTable("Vision")

try:
    print("\n👀 Monitoring Vision data... (Ctrl+C to stop)\n")
    
    while True:
        has_targets = vision.getBoolean("hasTargets", False)
        robot_count = vision.getNumber("robotCount", 0)
        
        if has_targets:
            print(f"🤖 Detected {int(robot_count)} robots:")
            
            for i in range(int(robot_count)):
                robot = vision.getSubTable(f"robots/{i}")
                label = robot.getString("label", "UNKNOWN")
                distance = robot.getNumber("distance", -1)
                angle = robot.getNumber("angle", 0)
                confidence = robot.getNumber("confidence", 0)
                track_id = robot.getNumber("trackId", -1)
                
                print(f"  [{i}] {label} (ID:{int(track_id)}) - "
                      f"{distance:.2f}m @ {angle:.1f}° "
                      f"(conf: {confidence:.2f})")
        else:
            print("❌ No targets detected")
        
        print("-" * 60)
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\n\n🛑 Shutting down...")
    NetworkTables.shutdown()
