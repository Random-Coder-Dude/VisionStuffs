"""
NT4 Client - Publishes fake robot data to existing NT4 server
Run this to test NT4 connection without camera
"""

import ntcore
import time
import random

SERVER_IP = "127.0.0.1"  # Change to your NT4 server IP

print(f"🔌 Connecting to NT4 server at {SERVER_IP}...")
inst = ntcore.NetworkTableInstance.getDefault()
inst.setServer(SERVER_IP)
inst.startClient4("fake-vision-client")

print("📡 Waiting for connection...")
while not inst.isConnected():
    time.sleep(0.1)

print("✅ Connected! Publishing fake robot data...")

vision = inst.getTable("Vision")

try:
    frame = 0
    while True:
        # Random 0-3 robots
        num_robots = random.randint(0, 3)
        
        if num_robots == 0:
            vision.putBoolean("hasTargets", False)
            vision.putNumber("robotCount", 0)
            vision.putNumber("redCount", 0)
            vision.putNumber("blueCount", 0)
        else:
            red_count = 0
            blue_count = 0
            
            vision.putBoolean("hasTargets", True)
            vision.putNumber("robotCount", num_robots)
            vision.putNumber("timestamp", time.time())
            vision.putNumber("frameNumber", frame)
            
            for i in range(num_robots):
                is_red = random.choice([True, False])
                if is_red:
                    red_count += 1
                    label = f"RED{red_count}"
                else:
                    blue_count += 1
                    label = f"BLUE{blue_count}"
                
                robot = vision.getSubTable(f"robots/{i}")
                robot.putString("label", label)
                robot.putNumber("x", random.uniform(100, 540))
                robot.putNumber("y", random.uniform(100, 380))
                robot.putNumber("distance", random.uniform(1.0, 5.0))
                robot.putNumber("angle", random.uniform(-30, 30))
                robot.putNumber("confidence", random.uniform(0.8, 0.99))
                robot.putNumber("trackId", i+1)
                robot.putBoolean("isRed", is_red)
                robot.putBoolean("isBlue", not is_red)
            
            vision.putNumber("redCount", red_count)
            vision.putNumber("blueCount", blue_count)
        
        if frame % 30 == 0:
            print(f"📡 Frame {frame}: {num_robots} robots published")
        
        frame += 1
        time.sleep(0.033)  # 30 FPS
        
except KeyboardInterrupt:
    print("\n🛑 Shutting down...")
    inst.stopClient()
