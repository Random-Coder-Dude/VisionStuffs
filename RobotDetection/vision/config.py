import json
import os
import logging
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)

# Where the config is persisted on disk.
# Relative to wherever main.py is run from (project root).
CONFIG_FILE = "vision_config.json"


# ============================================================
# DATACLASSES - Each group of related settings is its own class.
# Using @dataclass auto-generates __init__, __repr__, etc.
# Using default values means you get sane settings with zero config.
# ============================================================

@dataclass
class ColorConfig:
    """
    Controls how aggressively the color filter isolates red/blue bumpers.
    
    The detector checks: is this pixel's red channel >= (green * red_factor)?
    Higher factor = stricter, only catches very saturated red/blue.
    Lower factor = looser, catches more but may false-positive on skin tones, etc.
    These are tunable live via the UI sliders -> /api/config POST -> validate_and_set_config().
    """
    red_factor: float = 2.0
    blue_factor: float = 2.0


@dataclass
class BumperConfig:
    """
    Filters connected components (blobs) found in the color masks.
    After color detection + morphology cleanup, we have blobs of red/blue pixels.
    These settings decide which blobs are actually bumper-shaped.
    
    min_area:   Ignore blobs smaller than this (pixels^2). Filters out noise/reflections.
    min_aspect: Minimum width/height ratio. Bumpers are wide, so aspect >= 1.5 eliminates
                tall narrow blobs that aren't bumpers.
    max_aspect: Maximum width/height ratio. Very wide blobs might be reflections on the floor.
    """
    min_area: int = 1000
    min_aspect: float = 1.5
    max_aspect: float = 4.5


@dataclass
class MorphConfig:
    """
    Controls morphological closing applied to color masks before blob detection.
    
    Morphological closing = dilate then erode. It fills small holes in blobs and
    connects nearby blobs that are part of the same bumper but have gaps
    (e.g., from a black number on a red bumper breaking up the red region).
    
    kernel_size: Size of the structuring element (must be odd). Larger = more aggressive fill.
    iterations:  How many times to apply the operation. More = more fill but slower.
    """
    kernel_size: int = 7
    iterations: int = 1


@dataclass
class MetalConfig:
    """
    Controls the metallic detection used to CONFIRM robot detections.

    The pipeline finds colored bumper blobs, then looks ABOVE each bumper for
    a metallic robot body using Difference of Gaussians (DoG). This prevents
    false positives from colored walls, floor, or other non-robot objects.

    threshold:                  Fraction of pixels in the above-bumper search region
                                that must exceed the DoG threshold to confirm a robot.
                                Lower = more permissive, higher = stricter.
    search_height_multiplier:   Height of the search region above the bumper, as a
                                multiple of the bumper height. 1.0 = same height as bumper.
    dog_sigma1:                 Sigma for the fine-detail Gaussian blur in DoG.
                                Smaller = sensitive to finer edges/texture.
    dog_sigma2:                 Sigma for the coarse Gaussian blur in DoG.
                                Must be > dog_sigma1. Wider gap = more selective for
                                coarser texture. Good starting point: dog_sigma2 = 2x dog_sigma1.
    """
    threshold: float = 0.25
    search_height_multiplier: float = 1.0
    dog_sigma1: float = 1.0
    dog_sigma2: float = 1.6  # Paper recommends k = sigma2/sigma1 = 1.6
    dog_p: float = 20.0     # Edge emphasis. Paper suggests ~20 for strong edges.
    dog_epsilon: float = 0.5 # Threshold level in [0,1] normalized space.
    dog_phi: float = 10.0   # Transition sharpness. Higher = harder threshold.


@dataclass
class DebugConfig:
    """
    Controls what gets drawn on the processed video stream.
    show_overlay: Draw FPS + timing breakdown text on the video frame.
    show_verbose: Reserved for extra debug info if needed.
    """
    show_overlay: bool = False
    show_verbose: bool = False


@dataclass
class PerformanceConfig:
    """
    fps_smoothing_factor: The alpha for the exponential moving average in metrics.py.
    Lower alpha = smoother FPS reading (reacts slower to changes).
    Higher alpha = more responsive FPS reading (more jittery).
    See RollingAverage in vision/metrics.py.
    """
    fps_smoothing_factor: float = 0.2


@dataclass
class CameraConfig:
    """
    Camera hardware settings passed to camera_loop() in camera.py.
    device: Linux device index. 0 = /dev/video0, 1 = /dev/video1, etc.
    width/height: Requested capture resolution. Driver may round to nearest supported.
    """
    device: int = 0
    width: int = 640
    height: int = 480


@dataclass
class ServerConfig:
    """
    Flask server bind settings used in main.py when calling app.run().
    host "0.0.0.0" means accept connections on all network interfaces
    (i.e., accessible from other machines on the network, not just localhost).
    """
    port: int = 1403
    host: str = "10.0.0.207"


@dataclass
class NetworkTablesConfig:
    """
    Settings for publishing detection data to the robot via NetworkTables 4.
    Used by vision/networktables_publisher.py.
    
    enabled:        If False, NT publisher skips all publish() calls entirely.
    team_number:    FRC team number (used for auto-discovery of roboRIO if server_ip is empty).
    server_ip:      If set, connect as NT4 client to this IP (e.g., robot's roboRIO).
                    If empty, start as NT4 server (robot connects to us instead).
    update_rate_hz: Target publish rate (currently informational, not enforced by a timer -
                    publish() is called once per processed frame in stream()).
    """
    enabled: bool = True
    team_number: int = 1403
    server_ip: str = "10.0.0.199"
    update_rate_hz: float = 30.0


@dataclass
class DetectorConfig:
    """
    Root config object. Holds one instance of each sub-config above.
    
    This is the single source of truth for all tuneable parameters.
    It's instantiated ONCE at module level in app.py and shared with:
        - BumperDetector (reads it every frame during detection)
        - All /api/config routes (read and write it)
        - NetworkTablesPublisher (reads camera dimensions at startup)
        - main.py (reads camera/server settings before starting threads)
    
    The config_lock in SharedState (state.py) protects this object when
    the /api/config POST route writes new values while detect() is running.
    """
    color: ColorConfig = None
    bumper: BumperConfig = None
    morph: MorphConfig = None
    metal: MetalConfig = None
    debug: DebugConfig = None
    performance: PerformanceConfig = None
    camera: CameraConfig = None
    server: ServerConfig = None
    networktables: NetworkTablesConfig = None

    def __post_init__(self):
        # __post_init__ is called by the auto-generated @dataclass __init__.
        # We initialize sub-configs here (instead of as field defaults) because
        # mutable default arguments in dataclasses require field(default_factory=...)
        # which is verbose. This achieves the same thing more cleanly.
        if self.color is None: self.color = ColorConfig()
        if self.bumper is None: self.bumper = BumperConfig()
        if self.morph is None: self.morph = MorphConfig()
        if self.metal is None: self.metal = MetalConfig()
        if self.debug is None: self.debug = DebugConfig()
        if self.performance is None: self.performance = PerformanceConfig()
        if self.camera is None: self.camera = CameraConfig()
        if self.server is None: self.server = ServerConfig()
        if self.networktables is None: self.networktables = NetworkTablesConfig()

    def to_dict(self):
        """
        Serialize the full config to a plain dict.
        asdict() from dataclasses recursively converts nested dataclasses too.
        Used by save() to write JSON and by from_dict() (reset path).
        """
        return {
            "color": asdict(self.color),
            "bumper": asdict(self.bumper),
            "morph": asdict(self.morph),
            "metal": asdict(self.metal),
            "debug": asdict(self.debug),
            "performance": asdict(self.performance),
            "camera": asdict(self.camera),
            "server": asdict(self.server),
            "networktables": asdict(self.networktables)
        }

    def from_dict(self, data):
        """
        Load values from a plain dict into the existing config object in-place.
        Used by load() (reads from JSON file) and reset_config() route (copies defaults).
        
        We iterate keys defensively with hasattr() checks so a corrupt or outdated
        config file with unknown keys doesn't crash the app - unknown keys are silently ignored.
        """
        try:
            sections = {
                "color": self.color,
                "bumper": self.bumper,
                "morph": self.morph,
                "metal": self.metal,
                "debug": self.debug,
                "performance": self.performance,
                "camera": self.camera,
                "server": self.server,
                "networktables": self.networktables
            }
            for section_name, section_obj in sections.items():
                if section_name in data:
                    for key, value in data[section_name].items():
                        if hasattr(section_obj, key):
                            setattr(section_obj, key, value)
        except Exception as e:
            logger.error(f"Error loading config from dict: {e}")

    def save(self, filepath=CONFIG_FILE):
        """
        Write the current config to JSON on disk.
        Called by update_config() after any slider change, and save_config() route.
        Returns True on success so the API route can report status.
        """
        try:
            with open(filepath, 'w') as f:
                json.dump(self.to_dict(), f, indent=2)
            logger.info(f"Configuration saved to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Error saving config to {filepath}: {e}")
            return False

    def load(self, filepath=CONFIG_FILE):
        """
        Load config from JSON on disk, if it exists.
        Called once at startup in app.py before anything else runs.
        If the file doesn't exist (first run), silently uses dataclass defaults.
        """
        if not os.path.exists(filepath):
            logger.info(f"No config file found at {filepath}, using defaults")
            return False
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            self.from_dict(data)
            logger.info(f"Configuration loaded from {filepath}")
            return True
        except Exception as e:
            logger.error(f"Error loading config from {filepath}: {e}")
            return False
