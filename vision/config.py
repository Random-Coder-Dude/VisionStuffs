import json
import os
import logging
from dataclasses import dataclass, asdict

logger = logging.getLogger(__name__)

CONFIG_FILE = "vision_config.json"

# ---------- Color detection ----------
@dataclass
class ColorConfig:
    red_factor: float = 1.0
    blue_factor: float = 1.0

# ---------- Bumper geometry ----------
@dataclass
class BumperConfig:
    min_area: int = 1000
    min_aspect: float = 1.5
    max_aspect: float = 4.0

# ---------- Morphology ----------
@dataclass
class MorphConfig:
    kernel_size: int = 7
    iterations: int = 1

# ---------- Metallic detection ----------
@dataclass
class MetalConfig:
    threshold: float = 0.25
    spread_weight: float = 0.7
    search_height_multiplier: float = 1.0

# ---------- Debug / UI ----------
@dataclass
class DebugConfig:
    show_overlay: bool = False
    show_verbose: bool = False

# ---------- Performance ----------
@dataclass
class PerformanceConfig:
    fps_smoothing_factor: float = 0.2

# ---------- Camera settings ----------
@dataclass
class CameraConfig:
    device: int = 0
    width: int = 640
    height: int = 480

# ---------- Server settings ----------
@dataclass
class ServerConfig:
    port: int = 1403
    host: str = "0.0.0.0"

# ---------- Main config ----------
@dataclass
class DetectorConfig:
    color: ColorConfig = None
    bumper: BumperConfig = None
    morph: MorphConfig = None
    metal: MetalConfig = None
    debug: DebugConfig = None
    performance: PerformanceConfig = None
    camera: CameraConfig = None
    server: ServerConfig = None
    
    def __post_init__(self):
        if self.color is None:
            self.color = ColorConfig()
        if self.bumper is None:
            self.bumper = BumperConfig()
        if self.morph is None:
            self.morph = MorphConfig()
        if self.metal is None:
            self.metal = MetalConfig()
        if self.debug is None:
            self.debug = DebugConfig()
        if self.performance is None:
            self.performance = PerformanceConfig()
        if self.camera is None:
            self.camera = CameraConfig()
        if self.server is None:
            self.server = ServerConfig()
    
    def to_dict(self):
        """Convert config to dictionary for JSON serialization"""
        return {
            "color": asdict(self.color),
            "bumper": asdict(self.bumper),
            "morph": asdict(self.morph),
            "metal": asdict(self.metal),
            "debug": asdict(self.debug),
            "performance": asdict(self.performance),
            "camera": asdict(self.camera),
            "server": asdict(self.server)
        }
    
    def from_dict(self, data):
        """Load config from dictionary"""
        try:
            if "color" in data:
                for key, value in data["color"].items():
                    if hasattr(self.color, key):
                        setattr(self.color, key, value)
            
            if "bumper" in data:
                for key, value in data["bumper"].items():
                    if hasattr(self.bumper, key):
                        setattr(self.bumper, key, value)
            
            if "morph" in data:
                for key, value in data["morph"].items():
                    if hasattr(self.morph, key):
                        setattr(self.morph, key, value)
            
            if "metal" in data:
                for key, value in data["metal"].items():
                    if hasattr(self.metal, key):
                        setattr(self.metal, key, value)
            
            if "debug" in data:
                for key, value in data["debug"].items():
                    if hasattr(self.debug, key):
                        setattr(self.debug, key, value)
            
            if "performance" in data:
                for key, value in data["performance"].items():
                    if hasattr(self.performance, key):
                        setattr(self.performance, key, value)
            
            if "camera" in data:
                for key, value in data["camera"].items():
                    if hasattr(self.camera, key):
                        setattr(self.camera, key, value)
            
            if "server" in data:
                for key, value in data["server"].items():
                    if hasattr(self.server, key):
                        setattr(self.server, key, value)
                        
        except Exception as e:
            logger.error(f"Error loading config from dict: {e}")
    
    def save(self, filepath=CONFIG_FILE):
        """Save config to JSON file"""
        try:
            with open(filepath, 'w') as f:
                json.dump(self.to_dict(), f, indent=2)
            logger.info(f"Configuration saved to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Error saving config to {filepath}: {e}")
            return False
    
    def load(self, filepath=CONFIG_FILE):
        """Load config from JSON file"""
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
