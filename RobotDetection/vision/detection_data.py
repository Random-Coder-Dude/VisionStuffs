from dataclasses import dataclass, field
from typing import List, Optional
import time


@dataclass
class RobotDetection:
    """
    Data for a single detected robot in one frame.

    Created by BumperDetector.confirm_robots() in detector.py after a bumper blob
    passes the metallic body check. Then passed to RobotTracker.update() which
    assigns a persistent track_id before the detection is stored in latest_detections.

    FLOW:
        detector.py: confirm_robots() creates RobotDetection(track_id=None)
            -> tracker.py: update() assigns track_id (e.g., 1 for RED1)
            -> detector.py: stores list in DetectionResult
            -> app.py: stream() passes DetectionResult to nt_publisher.publish()
            -> app.py: /api/detections returns it as JSON to any HTTP client
    """

    # Bounding box top-left corner and dimensions in pixels.
    # x,y = top-left; width,height = size of box.
    x: int
    y: int
    width: int
    height: int

    # Pixel coordinates of the center of the bounding box.
    # Precomputed here so callers don't have to recalculate every time.
    # Used by tracker.py for distance calculations between frames.
    center_x: float
    center_y: float

    # How confident we are this is a real robot. Derived from the fraction of
    # pixels above the metallic threshold in the search region above the bumper.
    # Range: 0.0 to 1.0. Published to NetworkTables so robot code can filter
    # low-confidence detections if needed.
    confidence: float

    is_red: bool   # True if this robot has a red bumper
    is_blue: bool  # True if this robot has a blue bumper

    # Assigned by RobotTracker.update() in tracker.py.
    # Persistent across frames - same physical robot keeps same ID until it
    # disappears for max_missing_frames frames.
    # None until tracker assigns it.
    track_id: Optional[int] = None

    # When this detection was created. Used by NetworkTables publisher
    # so the robot knows how stale the data is.
    timestamp: float = field(default_factory=time.time)

    def get_label(self) -> str:
        """
        Human-readable label combining color and track ID.
        e.g., track_id=1 on a red robot -> "RED1"
        Drawn on video by detector.py and published to NT by networktables_publisher.py.
        """
        if self.track_id is None:
            return "UNKNOWN"
        color = "RED" if self.is_red else "BLUE"
        return f"{color}{self.track_id}"

    def to_dict(self):
        """
        Serialize to plain dict for JSON responses from /api/detections in app.py
        and for NetworkTables publishing in networktables_publisher.py.
        All values explicitly cast to Python primitives (int/float/bool/str)
        to avoid JSON serialization issues with numpy types from OpenCV.
        """
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
    """
    All detections for a single processed frame.

    Created by BumperDetector.confirm_robots() and stored as detector.latest_detections.
    This is what gets passed to nt_publisher.publish() after each frame
    and returned by /api/detections as JSON.

    PROPERTIES (computed from self.robots list):
        robot_count: total robots seen this frame
        has_targets:  True if any robots detected
        red_robots:   filtered list of red team robots
        blue_robots:  filtered list of blue team robots
    """
    robots: List[RobotDetection]  # All detected robots this frame (red + blue combined)
    timestamp: float              # When this result was created (time.time())
    frame_number: int             # Monotonically increasing frame counter from detector.py

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
        """
        Serialize to plain dict for /api/detections JSON response in app.py.
        active_ids is added on top of this in app.py from detector.tracker.get_active_ids().
        """
        return {
            'robots': [r.to_dict() for r in self.robots],
            'robot_count': int(self.robot_count),
            'has_targets': bool(self.has_targets),
            'red_count': int(len(self.red_robots)),
            'blue_count': int(len(self.blue_robots)),
            'timestamp': float(self.timestamp),
            'frame_number': int(self.frame_number)
        }
