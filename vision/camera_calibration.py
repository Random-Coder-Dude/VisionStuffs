import numpy as np

class CameraCalibration:
    """
    Calculate real-world positions from pixel coordinates
    Requires camera calibration (focal length, image center)
    """
    def __init__(self, 
                 focal_length_px: float = 500.0,  # Focal length in pixels
                 image_width: int = 640,
                 image_height: int = 480,
                 camera_height_m: float = 0.5,  # Camera height off ground (meters)
                 camera_angle_deg: float = 0.0):  # Camera tilt angle (degrees)
        
        self.focal_length = focal_length_px
        self.cx = image_width / 2.0  # Principal point x
        self.cy = image_height / 2.0  # Principal point y
        self.camera_height = camera_height_m
        self.camera_angle = np.radians(camera_angle_deg)
    
    def pixel_to_angle(self, pixel_x: float, pixel_y: float) -> tuple:
        """
        Convert pixel coordinates to angles (horizontal, vertical)
        Returns: (angle_horizontal_deg, angle_vertical_deg)
        """
        # Horizontal angle (yaw) - positive = right
        angle_h = np.arctan2(pixel_x - self.cx, self.focal_length)
        
        # Vertical angle (pitch) - positive = down
        angle_v = np.arctan2(pixel_y - self.cy, self.focal_length)
        
        return np.degrees(angle_h), np.degrees(angle_v)
    
    def estimate_distance(self, pixel_y: float, target_height_m: float = 0.3) -> float:
        """
        Estimate distance to target based on pixel y-coordinate
        Assumes target is on the ground
        
        Args:
            pixel_y: Y coordinate in pixels (0 = top of image)
            target_height_m: Height of target off ground
        
        Returns:
            Estimated distance in meters
        """
        # Simple pinhole camera model
        angle_v_rad = np.arctan2(pixel_y - self.cy, self.focal_length)
        total_angle = self.camera_angle + angle_v_rad
        
        if abs(total_angle) < 0.01:  # Avoid division by zero
            return 10.0  # Default far distance
        
        distance = (self.camera_height - target_height_m) / np.tan(total_angle)
        return max(0.1, abs(distance))  # Clamp to positive
    
    def add_pose_to_detection(self, detection):
        """
        Add distance and angle to a RobotDetection
        Modifies detection in-place
        """
        angle_h, angle_v = self.pixel_to_angle(detection.center_x, detection.center_y)
        distance = self.estimate_distance(detection.center_y)
        
        detection.angle_degrees = angle_h
        detection.distance_meters = distance
