import cv2
import time
import logging
from flask import Flask, Response, request, jsonify, render_template_string, make_response
from dataclasses import asdict, fields

from server.state import SharedState
from vision.detector import BumperDetector
from vision.config import DetectorConfig
from vision.system_monitor import SystemMonitor
from vision.networktables_publisher import NetworkTablesPublisher

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

state = SharedState()
config = DetectorConfig()

# Load saved configuration
config.load()

# Disable overlay by default for graph-based UI
config.debug.show_overlay = False

detector = BumperDetector(config)

# Start system monitor
system_monitor = SystemMonitor()
system_monitor.start()

# Initialize NetworkTables publisher
nt_publisher = NetworkTablesPublisher(
    enabled=config.networktables.enabled,
    team_number=config.networktables.team_number,
    server_ip=config.networktables.server_ip
)

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
                # Publish detections to NetworkTables
                nt_publisher.publish(detector.latest_detections)

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

# ---------------- Debug Streams ----------------
def stream_debug_red():
    """Stream red color mask"""
    while True:
        red_mask = detector.get_red_mask()
        if red_mask is None:
            time.sleep(0.01)
            continue

        try:
            ret, jpg = cv2.imencode(".jpg", red_mask)
            if not ret:
                time.sleep(0.01)
                continue
                
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   jpg.tobytes() + b"\r\n")
        except Exception as e:
            logger.error(f"Red mask stream error: {e}")
            time.sleep(0.01)

def stream_debug_blue():
    """Stream blue color mask"""
    while True:
        blue_mask = detector.get_blue_mask()
        if blue_mask is None:
            time.sleep(0.01)
            continue

        try:
            ret, jpg = cv2.imencode(".jpg", blue_mask)
            if not ret:
                time.sleep(0.01)
                continue
                
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   jpg.tobytes() + b"\r\n")
        except Exception as e:
            logger.error(f"Blue mask stream error: {e}")
            time.sleep(0.01)

def stream_debug_metallic():
    """Stream metallic detection buffer"""
    while True:
        metallic = detector.get_metallic_buffer()
        if metallic is None:
            time.sleep(0.01)
            continue

        try:
            ret, jpg = cv2.imencode(".jpg", metallic)
            if not ret:
                time.sleep(0.01)
                continue
                
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   jpg.tobytes() + b"\r\n")
        except Exception as e:
            logger.error(f"Metallic stream error: {e}")
            time.sleep(0.01)

@app.route("/debug/red")
def debug_red():
    return Response(stream_debug_red(),
        mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/debug/blue")
def debug_blue():
    return Response(stream_debug_blue(),
        mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/debug/metallic")
def debug_metallic():
    return Response(stream_debug_metallic(),
        mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------- Stats API ----------------
@app.route("/api/stats", methods=["GET"])
def get_stats():
    """Get current performance and system stats"""
    fps_value = detector.fps_avg.value if detector.fps_avg.value is not None else 0.0
    
    return jsonify({
        "performance": {
            "fps": round(fps_value, 1),
            "frame_time_ms": round(detector.last_timings["total"] * 1000, 2),
            "timings": {
                "color_ms": round(detector.last_timings["color_masks"] * 1000, 2),
                "metallic_ms": round(detector.last_timings["metallic"] * 1000, 2),
                "morph_ms": round(detector.last_timings["morphology"] * 1000, 2),
                "bbox_ms": round(detector.last_timings["bbox"] * 1000, 2),
                "robot_ms": round(detector.last_timings["robot"] * 1000, 2)
            }
        },
        "system": system_monitor.get_stats()
    })

# ---------------- Detection API ----------------
@app.route("/api/detections", methods=["GET"])
def get_detections():
    """Get latest detection results with tracking IDs"""
    try:
        if detector.latest_detections is None:
            return jsonify({
                "has_targets": False,
                "robot_count": 0,
                "red_count": 0,
                "blue_count": 0,
                "robots": [],
                "timestamp": time.time(),
                "frame_number": 0,
                "active_ids": {"red": [], "blue": []}
            })
        
        result = detector.latest_detections.to_dict()
        result["active_ids"] = detector.tracker.get_active_ids()
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error in get_detections: {e}", exc_info=True)
        return jsonify({"error": str(e)}), 500

@app.route("/api/tracking/reset", methods=["POST"])
def reset_tracking():
    """Reset tracking state (clear all IDs)"""
    detector.tracker.reset()
    return jsonify({"status": "ok", "message": "Tracking reset"})

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

# ---------------- UI with Charts ----------------
HTML_TEMPLATE = """
<!doctype html>
<html>
<head>
  <title>Robot Vision Dashboard</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
  <style>
    * { box-sizing: border-box; }
    body {
      background:#0b0b0b;
      color:#eee;
      font-family:'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      margin:0;
      padding:0;
      display:grid;
      grid-template-columns: 350px 1fr 400px;
      grid-template-rows: auto 1fr;
      height:100vh;
      gap:0;
    }
    
    /* Header */
    #header {
      grid-column: 1 / -1;
      background:#1a1a1a;
      padding:15px 20px;
      border-bottom:2px solid #00ff88;
      display:flex;
      justify-content:space-between;
      align-items:center;
    }
    #header h1 {
      margin:0;
      font-size:24px;
      color:#00ff88;
    }
    
    /* Left sidebar - Controls */
    #controls {
      padding:20px;
      border-right:1px solid #333;
      overflow-y:auto;
      background:#0f0f0f;
    }
    
    /* Center - Video */
    #video-container {
      display:flex;
      flex-direction:column;
      align-items:center;
      justify-content:center;
      padding:20px;
      background:#000;
    }
    #video-container img {
      max-width:100%;
      max-height:calc(100vh - 200px);
      border:2px solid #333;
      border-radius:4px;
    }
    
    /* Right sidebar - Graphs */
    #graphs {
      padding:20px;
      border-left:1px solid #333;
      overflow-y:auto;
      background:#0f0f0f;
    }
    
    /* Chart containers */
    .chart-container {
      margin-bottom:25px;
      background:#1a1a1a;
      padding:15px;
      border-radius:8px;
      border:1px solid #333;
    }
    .chart-title {
      color:#00ff88;
      font-size:14px;
      font-weight:600;
      margin-bottom:10px;
      text-transform:uppercase;
      letter-spacing:0.5px;
    }
    canvas {
      max-height:180px !important;
    }
    
    /* Controls styling */
    h3 {
      color:#00ff88;
      margin:20px 0 10px 0;
      font-size:14px;
      text-transform:uppercase;
      border-bottom:1px solid #333;
      padding-bottom:8px;
      letter-spacing:0.5px;
    }
    .group { margin-bottom:15px; }
    label {
      font-size:13px;
      display:block;
      margin-bottom:5px;
      color:#ccc;
    }
    input[type=range] {
      width:100%;
      height:4px;
      background:#333;
      outline:none;
      border-radius:2px;
    }
    input[type=range]::-webkit-slider-thumb {
      width:14px;
      height:14px;
      background:#00ff88;
      cursor:pointer;
      border-radius:50%;
    }
    input[type=number], input[type=text] {
      width:100%;
      padding:8px;
      background:#1a1a1a;
      color:#eee;
      border:1px solid #333;
      border-radius:4px;
      font-size:13px;
    }
    input[type=checkbox] {
      margin-right:8px;
      width:16px;
      height:16px;
    }
    .value {
      float:right;
      color:#00ff88;
      font-weight:600;
      font-size:13px;
    }
    button {
      padding:10px 16px;
      background:#1a5f7a;
      color:#fff;
      border:none;
      cursor:pointer;
      border-radius:4px;
      font-size:13px;
      font-weight:600;
      transition:background 0.2s;
    }
    button:hover { background:#2a7f9a; }
    .button-group { margin-top:20px; }
    .status {
      padding:10px;
      margin-top:10px;
      border-radius:4px;
      background:#1a3a1a;
      color:#8f8;
      display:none;
      font-size:13px;
    }
    
    /* Stats boxes */
    .stat-box {
      display:flex;
      justify-content:space-between;
      padding:10px;
      background:#1a1a1a;
      border-radius:4px;
      margin-bottom:8px;
      border-left:3px solid #00ff88;
    }
    .stat-label {
      color:#999;
      font-size:12px;
    }
    .stat-value {
      color:#00ff88;
      font-weight:600;
      font-size:14px;
    }
  </style>
</head>
<body>

<div id="header">
  <h1>🤖 Vision Dashboard</h1>
  <div style="color:#999; font-size:14px;">Real-time Robot Detection</div>
</div>

<div id="controls">
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

    <label>Search Height <span id="sh" class="value">1.0</span></label>
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

    <label>Min Aspect <span id="bmin" class="value">1.5</span></label>
    <input type="range" min="0.5" max="5" step="0.1" value="1.5"
      oninput="set('bumper.min_aspect', this.value, 'bmin')">

    <label>Max Aspect <span id="bmax" class="value">4.0</span></label>
    <input type="range" min="1" max="10" step="0.5" value="4.0"
      oninput="set('bumper.max_aspect', this.value, 'bmax')">
  </div>

  <div class="button-group">
    <button onclick="resetConfig()">Reset to Defaults</button>
  </div>
  
  <div id="status" class="status">Saved!</div>
</div>

<div id="video-container">
  <img src="/processed">
</div>

<div id="graphs">
  <h3 style="margin-top:0;">Performance Metrics</h3>
  
  <div class="stat-box">
    <span class="stat-label">FPS</span>
    <span class="stat-value" id="fps-value">0</span>
  </div>
  <div class="stat-box">
    <span class="stat-label">Frame Time</span>
    <span class="stat-value" id="frametime-value">0 ms</span>
  </div>
  
  <div class="chart-container">
    <div class="chart-title">FPS Over Time</div>
    <canvas id="fpsChart"></canvas>
  </div>

  <div class="chart-container">
    <div class="chart-title">Pipeline Timing Breakdown</div>
    <canvas id="timingChart"></canvas>
  </div>

  <h3>System Resources</h3>
  
  <div class="stat-box">
    <span class="stat-label">CPU Usage</span>
    <span class="stat-value" id="cpu-value">0%</span>
  </div>
  <div class="stat-box">
    <span class="stat-label">Memory Usage</span>
    <span class="stat-value" id="mem-value">0%</span>
  </div>
  <div class="stat-box" id="temp-box" style="display:none;">
    <span class="stat-label">Temperature</span>
    <span class="stat-value" id="temp-value">0°C</span>
  </div>

  <div class="chart-container">
    <div class="chart-title">CPU & Memory Usage</div>
    <canvas id="systemChart"></canvas>
  </div>
</div>

<script>
// Chart.js default config
Chart.defaults.color = '#999';
Chart.defaults.borderColor = '#333';
Chart.defaults.font.family = "'Segoe UI', sans-serif";

const maxDataPoints = 60;

// FPS Chart
const fpsChart = new Chart(document.getElementById('fpsChart'), {
  type: 'line',
  data: {
    labels: Array(maxDataPoints).fill(''),
    datasets: [{
      label: 'FPS',
      data: Array(maxDataPoints).fill(0),
      borderColor: '#00ff88',
      backgroundColor: 'rgba(0, 255, 136, 0.1)',
      borderWidth: 2,
      tension: 0.4,
      fill: true
    }]
  },
  options: {
    responsive: true,
    maintainAspectRatio: true,
    animation: false,
    plugins: { legend: { display: false } },
    scales: {
      y: {
        beginAtZero: true,
        max: 80,
        grid: { color: '#222' }
      },
      x: { display: false }
    }
  }
});

// Timing Breakdown Chart
const timingChart = new Chart(document.getElementById('timingChart'), {
  type: 'bar',
  data: {
    labels: ['Color', 'Metallic', 'Morph', 'BBox', 'Robot'],
    datasets: [{
      label: 'Time (ms)',
      data: [0, 0, 0, 0, 0],
      backgroundColor: ['#ff6384', '#36a2eb', '#ffce56', '#4bc0c0', '#9966ff'],
      borderWidth: 0
    }]
  },
  options: {
    responsive: true,
    maintainAspectRatio: true,
    animation: false,
    plugins: { legend: { display: false } },
    scales: {
      y: {
        beginAtZero: true,
        grid: { color: '#222' }
      },
      x: { grid: { display: false } }
    }
  }
});

// System Resources Chart
const systemChart = new Chart(document.getElementById('systemChart'), {
  type: 'line',
  data: {
    labels: Array(maxDataPoints).fill(''),
    datasets: [
      {
        label: 'CPU %',
        data: Array(maxDataPoints).fill(0),
        borderColor: '#ff6384',
        backgroundColor: 'rgba(255, 99, 132, 0.1)',
        borderWidth: 2,
        tension: 0.4,
        fill: true
      },
      {
        label: 'Memory %',
        data: Array(maxDataPoints).fill(0),
        borderColor: '#36a2eb',
        backgroundColor: 'rgba(54, 162, 235, 0.1)',
        borderWidth: 2,
        tension: 0.4,
        fill: true
      }
    ]
  },
  options: {
    responsive: true,
    maintainAspectRatio: true,
    animation: false,
    plugins: {
      legend: {
        display: true,
        position: 'top',
        labels: { color: '#999', font: { size: 11 } }
      }
    },
    scales: {
      y: {
        beginAtZero: true,
        max: 100,
        grid: { color: '#222' }
      },
      x: { display: false }
    }
  }
});

// Update charts with new data
function updateCharts(stats) {
  // Update stat boxes
  document.getElementById('fps-value').innerText = stats.performance.fps;
  document.getElementById('frametime-value').innerText = stats.performance.frame_time_ms + ' ms';
  document.getElementById('cpu-value').innerText = stats.system.cpu_percent + '%';
  document.getElementById('mem-value').innerText = stats.system.memory_percent + '%';
  
  if (stats.system.temperature !== null) {
    document.getElementById('temp-box').style.display = 'flex';
    document.getElementById('temp-value').innerText = stats.system.temperature + '°C';
  }
  
  // Update FPS chart
  fpsChart.data.datasets[0].data.push(stats.performance.fps);
  fpsChart.data.datasets[0].data.shift();
  fpsChart.update('none');
  
  // Update timing chart
  const timings = stats.performance.timings;
  timingChart.data.datasets[0].data = [
    timings.color_ms,
    timings.metallic_ms,
    timings.morph_ms,
    timings.bbox_ms,
    timings.robot_ms
  ];
  timingChart.update('none');
  
  // Update system chart
  systemChart.data.datasets[0].data.push(stats.system.cpu_percent);
  systemChart.data.datasets[0].data.shift();
  systemChart.data.datasets[1].data.push(stats.system.memory_percent);
  systemChart.data.datasets[1].data.shift();
  systemChart.update('none');
}

// Poll stats every 100ms
setInterval(() => {
  fetch('/api/stats')
    .then(r => r.json())
    .then(updateCharts)
    .catch(e => console.error('Stats fetch error:', e));
}, 100);

// Config management
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
        showStatus('Reset!');
        setTimeout(() => location.reload(), 1000);
      });
  }
}

function showStatus(msg = 'Saved!') {
  const status = document.getElementById('status');
  status.innerText = msg;
  status.style.display = 'block';
  setTimeout(() => { status.style.display = 'none'; }, 1500);
}

// Load initial config
fetch('/api/config')
  .then(r => r.json())
  .then(cfg => {
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
