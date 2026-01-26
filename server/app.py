import cv2
import time
import logging
from flask import Flask, Response, request, jsonify, render_template_string, make_response
from dataclasses import asdict, fields

from server.state import SharedState
from vision.detector import BumperDetector
from vision.config import DetectorConfig

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

state = SharedState()
config = DetectorConfig()

# Load saved configuration
config.load()

detector = BumperDetector(config)

# ---------------- Streaming ----------------
def stream(processed=False):
    while True:
        with state.lock:
            frame = None if state.frame is None else state.frame.copy()

        if frame is None:
            time.sleep(0.01)
            continue

        try:
            if processed:
                frame = detector.detect(frame)

            ret, jpg = cv2.imencode(".jpg", frame)
            if not ret:
                logger.error("Failed to encode frame to JPEG")
                time.sleep(0.01)
                continue
                
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   jpg.tobytes() + b"\r\n")
        except Exception as e:
            logger.error(f"Stream error: {e}")
            time.sleep(0.01)


@app.route("/raw")
def raw():
    return Response(stream(False),
        mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/processed")
def processed():
    return Response(stream(True),
        mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------- Hot-reload config API ----------------
def validate_and_set_config(config_obj, key_parts, value):
    """Safely validate and set config values with type checking."""
    if not key_parts:
        return False
    
    obj = config_obj
    
    # Navigate to the parent object
    for part in key_parts[:-1]:
        if not hasattr(obj, part):
            return False
        obj = getattr(obj, part)
    
    # Check if final attribute exists
    final_key = key_parts[-1]
    if not hasattr(obj, final_key):
        return False
    
    # Get current value to check type
    current_value = getattr(obj, final_key)
    expected_type = type(current_value)
    
    # Type conversion and validation
    try:
        if expected_type == bool:
            converted_value = bool(value)
        elif expected_type == int:
            converted_value = int(value)
        elif expected_type == float:
            converted_value = float(value)
        elif expected_type == str:
            converted_value = str(value)
        else:
            converted_value = value
        
        # Set the value
        setattr(obj, final_key, converted_value)
        return True
    except (ValueError, TypeError):
        return False


@app.route("/api/config", methods=["POST"])
def update_config():
    data = request.json or {}
    updated = {}
    failed = {}

    with state.config_lock:
        for key, value in data.items():
            parts = key.split(".")
            
            if validate_and_set_config(config, parts, value):
                updated[key] = value
            else:
                failed[key] = "Invalid key or type"
        
        # Save config after successful updates
        if updated:
            config.save()
    
    return jsonify({
        "status": "ok" if not failed else "partial",
        "updated": updated,
        "failed": failed
    })

@app.route("/api/config", methods=["GET"])
def get_config():
    """Get all current configuration values"""
    with state.config_lock:
        return jsonify({
            "color": {
                "red_factor": config.color.red_factor,
                "blue_factor": config.color.blue_factor
            },
            "bumper": {
                "min_area": config.bumper.min_area,
                "min_aspect": config.bumper.min_aspect,
                "max_aspect": config.bumper.max_aspect
            },
            "morph": {
                "kernel_size": config.morph.kernel_size,
                "iterations": config.morph.iterations
            },
            "metal": {
                "threshold": config.metal.threshold,
                "spread_weight": config.metal.spread_weight,
                "search_height_multiplier": config.metal.search_height_multiplier
            },
            "debug": {
                "show_overlay": config.debug.show_overlay,
                "show_verbose": config.debug.show_verbose
            },
            "performance": {
                "fps_smoothing_factor": config.performance.fps_smoothing_factor
            },
            "camera": {
                "device": config.camera.device,
                "width": config.camera.width,
                "height": config.camera.height
            },
            "server": {
                "port": config.server.port,
                "host": config.server.host
            }
        })

@app.route("/api/config/save", methods=["POST"])
def save_config():
    """Manually save current configuration"""
    with state.config_lock:
        success = config.save()
    return jsonify({"status": "ok" if success else "error"})

@app.route("/api/config/reset", methods=["POST"])
def reset_config():
    """Reset configuration to defaults"""
    with state.config_lock:
        # Create new config with defaults
        new_config = DetectorConfig()
        
        # Copy defaults to current config
        config.from_dict(new_config.to_dict())
        
        # Save the reset config
        config.save()
    
    return jsonify({"status": "ok", "message": "Configuration reset to defaults"})

# ---------------- Simple UI ----------------
HTML_TEMPLATE = """
<!doctype html>
<html>
<head>
  <title>Robot Vision Tuning</title>
  <style>
    body {
      background:#0b0b0b;
      color:#eee;
      font-family:system-ui, sans-serif;
      display:flex;
      margin:0;
      padding:0;
    }
    #controls {
      width:400px;
      padding:20px;
      border-right:1px solid #333;
      overflow-y:auto;
      height:100vh;
    }
    #video-container {
      flex:1;
      display:flex;
      align-items:center;
      justify-content:center;
      padding:20px;
    }
    h2 { margin-top:0; color:#8fd; }
    h3 { color:#8fd; margin-top:20px; margin-bottom:10px; border-bottom:1px solid #333; padding-bottom:5px; }
    .group { margin-bottom:15px; }
    label { font-size:14px; display:block; margin-bottom:3px; }
    input[type=range] { width:100%; }
    input[type=number], input[type=text] { width:100%; padding:5px; background:#1a1a1a; color:#eee; border:1px solid #333; }
    input[type=checkbox] { margin-right:5px; }
    .value { float:right; color:#8fd; font-weight:bold; }
    img { max-width:100%; border:2px solid #333; }
    button { 
      padding:8px 15px; 
      background:#1a5f7a; 
      color:#fff; 
      border:none; 
      cursor:pointer; 
      border-radius:4px;
      margin-right:10px;
    }
    button:hover { background:#2a7f9a; }
    button.danger { background:#7a1a1a; }
    button.danger:hover { background:#9a2a2a; }
    .button-group { margin-top:20px; }
    .status { 
      padding:10px; 
      margin-top:10px; 
      border-radius:4px; 
      background:#1a3a1a;
      color:#8f8;
      display:none;
    }
  </style>
</head>
<body>

<div id="controls">
  <h2>🤖 Vision Tuning</h2>

  <h3>Color Detection</h3>
  <div class="group">
    <label>Red Factor <span id="rf" class="value">2.0</span></label>
    <input type="range" min="1" max="5" step="0.1" value="2.0"
      oninput="set('color.red_factor', this.value, 'rf')">

    <label>Blue Factor <span id="bf" class="value">1.7</span></label>
    <input type="range" min="1" max="5" step="0.1" value="1.7"
      oninput="set('color.blue_factor', this.value, 'bf')">
  </div>

  <h3>Metallic Detection</h3>
  <div class="group">
    <label>Threshold <span id="mt" class="value">0.25</span></label>
    <input type="range" min="0" max="1" step="0.02" value="0.25"
      oninput="set('metal.threshold', this.value, 'mt')">

    <label>Spread Weight <span id="sw" class="value">0.7</span></label>
    <input type="range" min="0" max="1" step="0.05" value="0.7"
      oninput="set('metal.spread_weight', this.value, 'sw')">

    <label>Search Height Multiplier <span id="sh" class="value">1.0</span></label>
    <input type="range" min="0.5" max="2" step="0.1" value="1.0"
      oninput="set('metal.search_height_multiplier', this.value, 'sh')">
  </div>

  <h3>Morphology</h3>
  <div class="group">
    <label>Kernel Size <span id="ks" class="value">7</span></label>
    <input type="range" min="3" max="21" step="2" value="7"
      oninput="set('morph.kernel_size', this.value, 'ks')">

    <label>Iterations <span id="mi" class="value">1</span></label>
    <input type="range" min="1" max="5" step="1" value="1"
      oninput="set('morph.iterations', this.value, 'mi')">
  </div>

  <h3>Bumper Detection</h3>
  <div class="group">
    <label>Min Area <span id="ba" class="value">1000</span></label>
    <input type="range" min="100" max="5000" step="100" value="1000"
      oninput="set('bumper.min_area', this.value, 'ba')">

    <label>Min Aspect Ratio <span id="bmin" class="value">1.5</span></label>
    <input type="range" min="0.5" max="5" step="0.1" value="1.5"
      oninput="set('bumper.min_aspect', this.value, 'bmin')">

    <label>Max Aspect Ratio <span id="bmax" class="value">4.0</span></label>
    <input type="range" min="1" max="10" step="0.5" value="4.0"
      oninput="set('bumper.max_aspect', this.value, 'bmax')">
  </div>

  <h3>Performance</h3>
  <div class="group">
    <label>FPS Smoothing Factor <span id="fps" class="value">0.2</span></label>
    <input type="range" min="0.05" max="0.5" step="0.05" value="0.2"
      oninput="set('performance.fps_smoothing_factor', this.value, 'fps')">
  </div>

  <h3>Debug</h3>
  <div class="group">
    <label>
      <input type="checkbox" id="overlay" checked onchange="setCheckbox('debug.show_overlay', this.checked)">
      Show Overlay
    </label>
    <label>
      <input type="checkbox" id="verbose" checked onchange="setCheckbox('debug.show_verbose', this.checked)">
      Show Verbose
    </label>
  </div>

  <h3>Camera Settings</h3>
  <div class="group">
    <label>Device ID</label>
    <input type="number" id="cam_dev" value="0" min="0" max="10" 
      onchange="set('camera.device', this.value, null)">

    <label>Width</label>
    <input type="number" id="cam_w" value="640" min="320" max="1920" 
      onchange="set('camera.width', this.value, null)">

    <label>Height</label>
    <input type="number" id="cam_h" value="480" min="240" max="1080" 
      onchange="set('camera.height', this.value, null)">
  </div>

  <h3>Server Settings</h3>
  <div class="group">
    <label>Port</label>
    <input type="number" id="srv_port" value="1403" min="1" max="65535" 
      onchange="set('server.port', this.value, null)">

    <label>Host</label>
    <input type="text" id="srv_host" value="0.0.0.0" 
      onchange="set('server.host', this.value, null)">
  </div>

  <div class="button-group">
    <button onclick="resetConfig()">Reset to Defaults</button>
  </div>
  
  <div id="status" class="status">Configuration saved!</div>
</div>

<div id="video-container">
  <img src="/processed" style="max-width:90%; max-height:90vh;">
</div>

<script>
function set(key, value, id) {
  if (id) {
    document.getElementById(id).innerText = value;
  }
  fetch('/api/config', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({[key]: parseFloat(value) || value})
  }).then(() => showStatus());
}

function setCheckbox(key, checked) {
  fetch('/api/config', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({[key]: checked})
  }).then(() => showStatus());
}

function resetConfig() {
  if (confirm('Reset all settings to defaults?')) {
    fetch('/api/config/reset', { method: 'POST' })
      .then(r => r.json())
      .then(() => {
        showStatus('Settings reset to defaults!');
        setTimeout(() => location.reload(), 1000);
      });
  }
}

function showStatus(msg = 'Configuration saved!') {
  const status = document.getElementById('status');
  status.innerText = msg;
  status.style.display = 'block';
  setTimeout(() => { status.style.display = 'none'; }, 2000);
}

// Load initial config and populate UI
fetch('/api/config')
  .then(r => r.json())
  .then(cfg => {
    console.log('Loaded config:', cfg);
    
    // Update UI with loaded values
    document.getElementById('rf').innerText = cfg.color.red_factor;
    document.querySelector('input[oninput*="color.red_factor"]').value = cfg.color.red_factor;
    
    document.getElementById('bf').innerText = cfg.color.blue_factor;
    document.querySelector('input[oninput*="color.blue_factor"]').value = cfg.color.blue_factor;
    
    document.getElementById('mt').innerText = cfg.metal.threshold;
    document.querySelector('input[oninput*="metal.threshold"]').value = cfg.metal.threshold;
    
    document.getElementById('sw').innerText = cfg.metal.spread_weight;
    document.querySelector('input[oninput*="metal.spread_weight"]').value = cfg.metal.spread_weight;
    
    document.getElementById('sh').innerText = cfg.metal.search_height_multiplier;
    document.querySelector('input[oninput*="search_height_multiplier"]').value = cfg.metal.search_height_multiplier;
    
    document.getElementById('ks').innerText = cfg.morph.kernel_size;
    document.querySelector('input[oninput*="morph.kernel_size"]').value = cfg.morph.kernel_size;
    
    document.getElementById('mi').innerText = cfg.morph.iterations;
    document.querySelector('input[oninput*="morph.iterations"]').value = cfg.morph.iterations;
    
    document.getElementById('ba').innerText = cfg.bumper.min_area;
    document.querySelector('input[oninput*="bumper.min_area"]').value = cfg.bumper.min_area;
    
    document.getElementById('bmin').innerText = cfg.bumper.min_aspect;
    document.querySelector('input[oninput*="bumper.min_aspect"]').value = cfg.bumper.min_aspect;
    
    document.getElementById('bmax').innerText = cfg.bumper.max_aspect;
    document.querySelector('input[oninput*="bumper.max_aspect"]').value = cfg.bumper.max_aspect;
    
    document.getElementById('fps').innerText = cfg.performance.fps_smoothing_factor;
    document.querySelector('input[oninput*="fps_smoothing_factor"]').value = cfg.performance.fps_smoothing_factor;
    
    document.getElementById('overlay').checked = cfg.debug.show_overlay;
    document.getElementById('verbose').checked = cfg.debug.show_verbose;
    
    document.getElementById('cam_dev').value = cfg.camera.device;
    document.getElementById('cam_w').value = cfg.camera.width;
    document.getElementById('cam_h').value = cfg.camera.height;
    
    document.getElementById('srv_port').value = cfg.server.port;
    document.getElementById('srv_host').value = cfg.server.host;
  });
</script>

</body>
</html>
"""

@app.route("/")
def index():
    response = make_response(HTML_TEMPLATE)
    response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate, max-age=0'
    response.headers['Pragma'] = 'no-cache'
    response.headers['Expires'] = '0'
    return response
