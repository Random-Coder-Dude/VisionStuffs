import logging
from typing import Optional
from vision.detection_data import DetectionResult

logger = logging.getLogger(__name__)

try:
    from networktables import NetworkTables
    NT_AVAILABLE = True
except ImportError:
    logger.warning("pynetworktables3 not installed. NetworkTables support disabled.")
    NT_AVAILABLE = False


class NetworkTablesPublisher:
    """
    Publishes robot detection data to NetworkTables for consumption by robot code
    """
    
    def __init__(self, enabled: bool = True, team_number: int = 1403, server_ip: str = "10.0.0.199"):
        """
        Args:
            enabled: Whether to publish to NetworkTables
            team_number: FRC team number (for auto-detecting roborio)
            server_ip: Manual server IP (leave empty for auto-detect)
        """
        self.enabled = enabled and NT_AVAILABLE
        self.team_number = team_number
        self.connected = False
        self.table = None
        
        if not NT_AVAILABLE:
            logger.warning("NetworkTables not available - install with: pip3 install pynetworktables3")
            self.enabled = False
            return
        
        if not self.enabled:
            logger.info("NetworkTables publishing disabled in config")
            return
        
        try:
            # Initialize NetworkTables
            NetworkTables.initialize(server=server_ip if server_ip else None)
            
            if not server_ip:
                # Auto-detect roborio
                logger.info(f"Connecting to roborio-{team_number}-frc.local or 10.{team_number//100}.{team_number%100}.2")
            else:
                logger.info(f"Connecting to NetworkTables server at {server_ip}")
            
            # Get vision table
            self.table = NetworkTables.getTable("Vision")
            
            # Set connection listener
            NetworkTables.addConnectionListener(self._connection_listener, immediateNotify=True)
            
            logger.info("NetworkTables publisher initialized")
            
        except Exception as e:
            logger.error(f"Failed to initialize NetworkTables: {e}")
            self.enabled = False
    
    def _connection_listener(self, connected, info):
        """Callback for connection status changes"""
        self.connected = connected
        if connected:
            logger.info(f"Connected to NetworkTables server: {info.remote_ip}")
        else:
            logger.warning("Disconnected from NetworkTables server")
    
    def publish(self, detections: Optional[DetectionResult]):
        """
        Publish detection results to NetworkTables
        
        Structure:
        /Vision/
          ├── hasTargets (bool)
          ├── robotCount (int)
          ├── redCount (int)
          ├── blueCount (int)
          ├── timestamp (float)
          ├── frameNumber (int)
          └── robots/
              ├── [0]/
              │   ├── x (float)
              │   ├── y (float)
              │   ├── width (int)
              │   ├── height (int)
              │   ├── confidence (float)
              │   ├── isRed (bool)
              │   ├── isBlue (bool)
              │   ├── trackId (int)
              │   ├── label (string)
              │   ├── distance (float)
              │   └── angle (float)
              └── [1]/
                  └── ...
        """
        if not self.enabled or self.table is None:
            return
        
        try:
            if detections is None or not detections.has_targets:
                # No targets
                self.table.putBoolean("hasTargets", False)
                self.table.putNumber("robotCount", 0)
                self.table.putNumber("redCount", 0)
                self.table.putNumber("blueCount", 0)
                return
            
            # Publish top-level data
            self.table.putBoolean("hasTargets", True)
            self.table.putNumber("robotCount", len(detections.robots))
            self.table.putNumber("redCount", len(detections.red_robots))
            self.table.putNumber("blueCount", len(detections.blue_robots))
            self.table.putNumber("timestamp", detections.timestamp)
            self.table.putNumber("frameNumber", detections.frame_number)
            
            # Publish individual robot data
            for i, robot in enumerate(detections.robots):
                robot_table = self.table.getSubTable(f"robots/{i}")
                
                robot_table.putNumber("x", robot.center_x)
                robot_table.putNumber("y", robot.center_y)
                robot_table.putNumber("width", robot.width)
                robot_table.putNumber("height", robot.height)
                robot_table.putNumber("confidence", robot.confidence)
                robot_table.putBoolean("isRed", robot.is_red)
                robot_table.putBoolean("isBlue", robot.is_blue)
                robot_table.putNumber("trackId", robot.track_id if robot.track_id is not None else -1)
                robot_table.putString("label", robot.get_label())
                
                # Optional pose data
                if robot.distance_meters is not None:
                    robot_table.putNumber("distance", robot.distance_meters)
                if robot.angle_degrees is not None:
                    robot_table.putNumber("angle", robot.angle_degrees)
            
            # Clear old robot entries if we have fewer robots now
            # (e.g., if we had 3 robots before but now only have 2)
            max_robots = 10  # Assume max 10 robots
            for i in range(len(detections.robots), max_robots):
                robot_table = self.table.getSubTable(f"robots/{i}")
                if robot_table.containsKey("x"):
                    # Clear this entry by setting trackId to -1
                    robot_table.putNumber("trackId", -1)
                else:
                    break  # No more old entries
            
        except Exception as e:
            logger.error(f"Error publishing to NetworkTables: {e}", exc_info=True)
    
    def is_connected(self) -> bool:
        """Check if connected to NetworkTables server"""
        return self.enabled and self.connected
    
    def shutdown(self):
        """Clean shutdown of NetworkTables"""
        if self.enabled and NT_AVAILABLE:
            try:
                NetworkTables.shutdown()
                logger.info("NetworkTables shut down")
            except Exception as e:
                logger.error(f"Error shutting down NetworkTables: {e}")
