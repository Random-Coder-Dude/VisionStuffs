import logging
from flask import Flask

from server.state import SharedState
from vision.config import DetectorConfig
from vision.pipeline.detector import BumperDetector
from vision.system_monitor import SystemMonitor
from vision.networktables_publisher import NetworkTablesPublisher

logging.basicConfig(level=logging.INFO)
logging.getLogger('werkzeug').setLevel(logging.ERROR)

# ── Singletons ────────────────────────────────────────────────────────────────
# Created once at import time. Shared by all route modules via import.

app = Flask(__name__, static_folder='static')

state = SharedState()

config = DetectorConfig()
config.load()
config.debug.show_overlay = False

detector = BumperDetector(config)

system_monitor = SystemMonitor()
system_monitor.start()

nt_publisher = NetworkTablesPublisher(
    enabled=config.networktables.enabled,
    team_number=config.networktables.team_number,
    server_ip=config.networktables.server_ip,
    camera_width=config.camera.width,
    camera_height=config.camera.height,
)

# ── Register route blueprints ─────────────────────────────────────────────────
from server.routes.stream    import bp as stream_bp     # noqa: E402
from server.routes.api_config import bp as config_bp    # noqa: E402
from server.routes.api_data  import bp as data_bp       # noqa: E402

app.register_blueprint(stream_bp)
app.register_blueprint(config_bp)
app.register_blueprint(data_bp)
