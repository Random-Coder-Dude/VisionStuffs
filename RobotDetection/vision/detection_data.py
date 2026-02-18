from dataclasses import dataclass, field
from typing import List, Optional
import time

@dataclass
class RobotDetection:
    """Single robot detection result"""
    # Bounding box in pixels
    x: int
    y: int
    width: int
    height: int
    
    # Center position in pixels
    center_x: float
    center_y: float
    
    # Confidence/quality metrics
    confidence: float  # 0-1, based on metallic score
    is_red: bool
    is_blue: bool
    
    # Tracking
    track_id: Optional[int] = None  # Unique ID (e.g., RED1 = 1, RED2 = 2)
    
    # Timestamp
    timestamp: float = field(default_factory=time.time)
    

    
    def get_label(self) -> str:
        """Get human-readable label like 'RED1' or 'BLUE2'"""
        if self.track_id is None:
            return "UNKNOWN"
        color = "RED" if self.is_red else "BLUE"
        return f"{color}{self.track_id}"
    
    def to_dict(self):
        """Convert to dictionary for JSON/API"""
        return {
            'x': float(self.center_x),
            'y': float(self.center_y),
            'width': int(self.width),
            'height': int(self.height),
            'confidence': float(self.confidence),
            'is_red': bool(self.is_red),
            'is_blue': bool(self.is_blue),
            'track_id': int(self.track_id) if self.track_id is not None else None,
            'label': self.get_label(),
            'timestamp': float(self.timestamp)
        }

@dataclass
class DetectionResult:
    """Complete detection result for a frame"""
    robots: List[RobotDetection]
    timestamp: float
    frame_number: int
    
    @property
    def robot_count(self) -> int:
        return len(self.robots)
    
    @property
    def has_targets(self) -> bool:
        return len(self.robots) > 0
    
    @property
    def red_robots(self) -> List[RobotDetection]:
        return [r for r in self.robots if r.is_red]
    
    @property
    def blue_robots(self) -> List[RobotDetection]:
        return [r for r in self.robots if r.is_blue]
    
    def to_dict(self):
        return {
            'robots': [r.to_dict() for r in self.robots],
            'robot_count': int(self.robot_count),
            'has_targets': bool(self.has_targets),
            'red_count': int(len(self.red_robots)),
            'blue_count': int(len(self.blue_robots)),
            'timestamp': float(self.timestamp),
            'frame_number': int(self.frame_number)
        }
