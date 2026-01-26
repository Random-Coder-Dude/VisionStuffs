from dataclasses import dataclass

# ---------- Color detection ----------
@dataclass
class ColorConfig:
    red_factor: float = 2.0
    blue_factor: float = 1.7

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
    show_overlay: bool = True


@dataclass
class DetectorConfig:
    color: ColorConfig = ColorConfig()
    bumper: BumperConfig = BumperConfig()
    morph: MorphConfig = MorphConfig()
    metal: MetalConfig = MetalConfig()
    debug: DebugConfig = DebugConfig()
