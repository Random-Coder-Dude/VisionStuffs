# Instructions to Add Debug Stream Support

## Step 1: Replace detector.py
Copy the contents of `vision/detector_with_debug.py` to `vision/detector.py`

Or simply:
```bash
cp vision/detector_with_debug.py vision/detector.py
```

## Step 2: Add debug stream endpoints to server/app.py

Add these new routes after the existing `/raw` and `/processed` routes:

```python
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
```

## Step 3: Update the HTML template to show debug streams

In the HTML_TEMPLATE variable, add a debug section. Find the graphs div and add after it:

```html
<div id="debug-panel" style="grid-column: 1 / -1; padding: 20px; background: #0f0f0f; border-top: 1px solid #333;">
  <h3 style="margin-top: 0;">🔍 Debug Views</h3>
  <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 20px;">
    <div class="stream-box">
      <h2 style="font-size: 1.1em;">Red Mask</h2>
      <img src="/debug/red" style="width: 100%; border: 2px solid #ff0000;">
    </div>
    <div class="stream-box">
      <h2 style="font-size: 1.1em;">Blue Mask</h2>
      <img src="/debug/blue" style="width: 100%; border: 2px solid #0000ff;">
    </div>
    <div class="stream-box">
      <h2 style="font-size: 1.1em;">Metallic Buffer</h2>
      <img src="/debug/metallic" style="width: 100%; border: 2px solid #888;">
    </div>
  </div>
</div>
```

And update the body grid:
```css
body {
  /* ... existing styles ... */
  grid-template-rows: auto 1fr auto;  /* Add 'auto' at end for debug row */
}
```

## Step 4: Test it!

1. Start your server:
```bash
python main.py
```

2. Open browser to `http://your-pi-ip:1403/`

3. You should now see 3 new debug views at the bottom showing:
   - **Red Mask**: White pixels where red is detected
   - **Blue Mask**: White pixels where blue is detected
   - **Metallic Buffer**: Brightness shows metallic score (brighter = more metallic)

## Direct URL Access

You can also view debug streams directly:
- Red mask: `http://your-pi-ip:1403/debug/red`
- Blue mask: `http://your-pi-ip:1403/debug/blue`
- Metallic: `http://your-pi-ip:1403/debug/metallic`

## What This Does

- **Red/Blue Masks**: Shows exactly what pixels are being detected as red/blue bumpers
  - White = detected
  - Black = not detected
  - Helps you tune `red_factor` and `blue_factor` parameters

- **Metallic Buffer**: Shows the metallic detection strength
  - Brighter = more metallic/shiny
  - Darker = less metallic
  - Helps you tune `metallic_threshold` and `spread_weight` parameters

## Performance Note

These debug streams are lightweight - they just grab the already-computed masks from the detector, so there's minimal performance impact.
