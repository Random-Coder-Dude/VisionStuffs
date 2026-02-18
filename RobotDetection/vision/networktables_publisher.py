import logging
from typing import Optional
from vision.detection_data import DetectionResult

logger = logging.getLogger(__name__)

try:
    import ntcore
    NT4_AVAILABLE = True
except ImportError:
    logger.warning("pyntcore not installed. Install with: pip3 install pyntcore")
    NT4_AVAILABLE = False


class NetworkTablesPublisher:
    """
    Publishes robot detection data to NetworkTables using NT4 (pyntcore)
    """
    
    def __init__(self, enabled: bool = True, team_number: int = 1403, server_ip: str = "", 
                 camera_width: int = 640, camera_height: int = 480):
        """
        Args:
            enabled: Whether to publish to NetworkTables
            team_number: FRC team number
            server_ip: Server IP (empty string = run as server)
            camera_width: Camera width for user calculations
            camera_height: Camera height for user calculations
        """
        self.enabled = enabled and NT4_AVAILABLE
        self.team_number = team_number
        self.connected = False
        self.nt4_inst = None
        self.vision_table = None
        self.camera_width = camera_width
        self.camera_height = camera_height
        
        if not NT4_AVAILABLE:
            logger.warning("NT4 not available")
            self.enabled = False
            return
        
        if not self.enabled:
            logger.info("NetworkTables publishing disabled in config")
            return
        
        try:
            self.nt4_inst = ntcore.NetworkTableInstance.getDefault()
            
            if not server_ip:
                # Run as server
                logger.info("NT4: Starting server on port 5810")
                self.nt4_inst.startServer()
            else:
                # Connect as client
                logger.info(f"NT4: Connecting to {server_ip}")
                self.nt4_inst.setServer(server_ip)
                self.nt4_inst.startClient4("vision-client")
            
            self.vision_table = self.nt4_inst.getTable("Vision")
            
            # Add connection listener (use prefix list, not table)
            listener_handle = self.nt4_inst.addListener(
                ["/"],  # Listen to root
                ntcore.EventFlags.kConnection,
                self._connection_listener
            )
            
            logger.info("NT4 initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize NT4: {e}")
            self.enabled = False
    
    def _connection_listener(self, event):
        """NT4 connection callback"""
        if event.flags & ntcore.EventFlags.kConnected:
            self.connected = True
            logger.info("NT4: Connected")
        elif event.flags & ntcore.EventFlags.kDisconnected:
            self.connected = False
            logger.warning("NT4: Disconnected")
    
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
          ├── cameraWidth (int)
          ├── cameraHeight (int)
          └── robots/
              ├── [0]/
              │   ├── centerX (float)     - Center of bounding box
              │   ├── centerY (float)
              │   ├── x (int)             - Top-left corner (raw BB)
              │   ├── y (int)
              │   ├── width (int)         - Bounding box dimensions
              │   ├── height (int)
              │   ├── confidence (float)  - Detection confidence (0-1)
              │   ├── isRed (bool)
              │   ├── isBlue (bool)
              │   ├── trackId (int)       - Persistent tracking ID
              │   ├── label (string)      - e.g., "RED1", "BLUE2"
              │   ├── distance (float)    - EMPTY - for user calculations
              │   └── angle (float)       - EMPTY - for user calculations
              └── [1]/
                  └── ...
        """
        if not self.enabled or self.vision_table is None:
            return
        
        try:
            if detections is None or not detections.has_targets:
                # No targets
                self.vision_table.putBoolean("hasTargets", False)
                self.vision_table.putNumber("robotCount", 0)
                self.vision_table.putNumber("redCount", 0)
                self.vision_table.putNumber("blueCount", 0)
                return
            
            # Publish top-level data
            self.vision_table.putBoolean("hasTargets", True)
            self.vision_table.putNumber("robotCount", len(detections.robots))
            self.vision_table.putNumber("redCount", len(detections.red_robots))
            self.vision_table.putNumber("blueCount", len(detections.blue_robots))
            self.vision_table.putNumber("timestamp", detections.timestamp)
            self.vision_table.putNumber("frameNumber", detections.frame_number)
            
            # Camera resolution for user calculations
            self.vision_table.putNumber("cameraWidth", self.camera_width)
            self.vision_table.putNumber("cameraHeight", self.camera_height)
            
            # Publish individual robot data
            for i, robot in enumerate(detections.robots):
                robot_table = self.vision_table.getSubTable(f"robots/{i}")
                
                # Center position
                robot_table.putNumber("centerX", robot.center_x)
                robot_table.putNumber("centerY", robot.center_y)
                
                # Raw bounding box (top-left corner + dimensions)
                robot_table.putNumber("x", robot.x)
                robot_table.putNumber("y", robot.y)
                robot_table.putNumber("width", robot.width)
                robot_table.putNumber("height", robot.height)
                robot_table.putNumber("confidence", robot.confidence)
                robot_table.putBoolean("isRed", robot.is_red)
                robot_table.putBoolean("isBlue", robot.is_blue)
                robot_table.putNumber("trackId", robot.track_id if robot.track_id is not None else -1)
                robot_table.putString("label", robot.get_label())
            
        except Exception as e:
            logger.error(f"Error publishing to NT4: {e}", exc_info=True)
    
    def is_connected(self) -> bool:
        """Check if connected to NetworkTables server"""
        return self.enabled and self.connected
    
    def shutdown(self):
        """Clean shutdown of NetworkTables"""
        if self.enabled and self.nt4_inst:
            try:
                self.nt4_inst.stopClient()
                logger.info("NT4 shut down")
            except Exception as e:
                logger.error(f"Error shutting down NT4: {e}")
