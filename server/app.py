import cv2
import time
from flask import Flask, Response, request, jsonify, render_template_string
from dataclasses import asdict

from server.state import SharedState
from vision.detector import BumperDetector
from vision.config import DetectorConfig

app = Flask(__name__)

state = SharedState()
config = DetectorConfig()
detector = BumperDetector(config)

# ---------------- Streaming ----------------
def stream(processed=False):
    while True:
        with state.lock:
            frame = None if state.frame is None else state.frame.copy()

        if frame is None:
            time.sleep(0.01)
            continue

        if processed:
            frame = detector.detect(frame)

        _, jpg = cv2.imencode(".jpg", frame)
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" +
               jpg.tobytes() + b"\r\n")

@app.route("/raw")
def raw():
    return Response(stream(False),
        mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/processed")
def processed():
    return Response(stream(True),
        mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------- Hot-reload config API ----------------
@app.route("/api/config", methods=["POST"])
def update_config():
    data = request.json or {}

    for key, value in data.items():
        parts = key.split(".")
        obj = config
        for p in parts[:-1]:
            obj = getattr(obj, p)
        setattr(obj, parts[-1], value)

    return jsonify({"status": "ok"})

# ---------------- Simple UI ----------------
@app.route("/")
def index():
    return render_template_string("""
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
    }
    #controls {
      width:380px;
      padding:20px;
      border-right:1px solid #333;
    }
    h2 { margin-top:0; }
    .group { margin-bottom:20px; }
    label { font-size:14px; }
    input[type=range] { width:100%; }
    .value { float:right; color:#8fd; }
  </style>
</head>
<body>

<div id="controls">
  <h2>🤖 Vision Tuning</h2>

  <div class="group">
    <h4>Color</h4>
    <label>Red Factor <span id="rf" class="value">2.0</span></label>
    <input type="range" min="1" max="5" step="0.1" value="2.0"
      oninput="set('color.red_factor', this.value, 'rf')">

    <label>Blue Factor <span id="bf" class="value">1.7</span></label>
    <input type="range" min="1" max="5" step="0.1" value="1.7"
      oninput="set('color.blue_factor', this.value, 'bf')">
  </div>

  <div class="group">
    <h4>Metal</h4>
    <label>Threshold <span id="mt" class="value">0.25</span></label>
    <input type="range" min="0" max="1" step="0.02" value="0.25"
      oninput="set('metal.threshold', this.value, 'mt')">

    <label>Search Height <span id="sh" class="value">1.0</span></label>
    <input type="range" min="0.5" max="2" step="0.1" value="1.0"
      oninput="set('metal.search_height_multiplier', this.value, 'sh')">
  </div>

  <div class="group">
    <h4>Morphology</h4>
    <label>Kernel Size <span id="ks" class="value">7</span></label>
    <input type="range" min="3" max="21" step="2" value="7"
      oninput="set('morph.kernel_size', this.value, 'ks')">
  </div>
</div>

<img src="/processed" width="800">

<script>
function set(key, value, id) {
  document.getElementById(id).innerText = value;
  fetch('/api/config', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({[key]: parseFloat(value)})
  });
}
</script>

</body>
</html>
""")
