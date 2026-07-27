# Kinect to MadMapper OSC Driver

A Python toolkit for connecting a Kinect (v1 Xbox 360, or v2 Xbox One) to MadMapper for interactive projection mapping — tracking people or detecting touch on a projected surface, and sending the result over OSC.

Two scripts:
*   **`kinect_osc_driver.py`** — the production driver. Tracks bodies or detects touch, sends OSC to MadMapper.
*   **`kinect_point_cloud.py`** — a live 3D point-cloud viewer (Kinect v2 only) for visually sanity-checking the sensor and its framing.

Runs on **macOS** (Apple Silicon or Intel, via `libfreenect`/`libfreenect2`) and **Windows** (Kinect v2 only, via the official Kinect for Windows SDK).

---

## macOS Setup

### 1. Hardware Requirements
*   Kinect v1 (Xbox 360) or Kinect v2 (Xbox One)
*   **Kinect AC Power Adapter** — the USB cable alone does not provide enough power.
*   For Kinect v2: a USB 3.0/USB4 port or high-bandwidth adapter.

### 2. Install System Dependencies
Install Homebrew if you haven't already:
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### 3. Build the drivers
`build_drivers.sh` builds `libfreenect`/`libfreenect2` and their Python bindings from source into `_libs/`, and sets up the `venv`:
```bash
./build_drivers.sh
```

### 4. Run
```bash
./run_driver.sh --camera v2 --debug
./run_point_cloud.sh
```

---

## Windows Setup (Kinect v2 only)

Windows uses the **official Kinect for Windows SDK 2.0** instead of `libfreenect2` — it's the natively-supported driver on this platform, so there's no C++ build step required.

### 1. Install the Kinect for Windows SDK 2.0
Download and install it from Microsoft (search "Kinect for Windows SDK 2.0"). This installs the drivers and runtime the sensor needs.

### 2. Set up Python
```bat
python -m venv venv
venv\Scripts\activate
pip install -r requirements-windows.txt
```

### 3. Run
```bat
run_driver.bat --camera v2 --debug
run_point_cloud.bat
```

Kinect v1 is not supported on Windows in this repo (no Windows backend has been written for it).

---

## Running the Driver

The driver has two modes:

### Track mode (default)
Tracks up to 3 people/blobs within a depth range and sends their position.
```bash
./run_driver.sh --camera v2 --min-depth 800 --max-depth 3000 --min-area 4000 --debug
```
*   Kinect v1 depth values range from `0` to `2047`.
*   Kinect v2 depth values represent distance in millimeters (e.g. `500` to `3000` mm).

### Touch mode
For installations where people physically touch a projected surface (e.g. a building facade) to trigger generative animations. Rather than a fixed depth range, touch mode calibrates against the surface itself, so a "touch" is detected as something entering a thin zone just in front of it — not just anyone standing in the room.

Mount the Kinect **off the surface, facing it from a distance** (roughly 2-3 meters back for Kinect v2, centered on the projected area) rather than flush against it — a touching hand needs to stay within the sensor's reliable range (Kinect v2: ~0.5m-4.5m), which won't hold if the sensor sits right at the surface.

```bash
./run_driver.sh --camera v2 --mode touch --debug
```

On startup it calibrates the empty wall for a moment — keep the area clear of people until you see "Wall calibration complete."

Tune the touch zone thickness if needed:
```bash
./run_driver.sh --camera v2 --mode touch --touch-min-offset 20 --touch-max-offset 150
```
(Kinect v2 values are in millimeters; Kinect v1's raw `0`-`2047` range is not physical distance, so these defaults will need re-tuning for `--camera v1`.)

Press `q` in the debug window or `Ctrl+C` in the terminal to cleanly stop the stream.

---

## OSC Address Map

The driver sends normalized coordinates (`0.0` to `1.0` scaling matching the respective camera aspect resolutions) to the following addresses in MadMapper:

**Track mode (default):**
*   `/kinect/person1/x` (Horizontal centroid)
*   `/kinect/person1/y` (Vertical centroid)
*   `/kinect/person1/depth` (Raw depth of center point)

Up to 3 bodies can be tracked simultaneously (`person1`, `person2`, `person3`).

**Touch mode:**
*   `/kinect/touch1/x` (Horizontal touch position, sent continuously while active)
*   `/kinect/touch1/y` (Vertical touch position)
*   `/kinect/touch1/active` (`1` while touching, `0` sent once when the touch ends)

Up to 3 simultaneous touches by default (`touch1`, `touch2`, `touch3`; adjust with `--max-touches`).

## How to use in MadMapper

1. Ensure MadMapper is open and OSC input is enabled (Preferences -> OSC -> check "Enable OSC Input", note the Port, usually 8000).
2. Run the driver (see above).
3. In MadMapper, use its OSC control-binding workflow (e.g. `Edit -> Edit OSC Controls`, naming varies by version) to assign an incoming address like `/kinect/touch1/x` to a parameter — position, opacity, a trigger, etc.

## Point Cloud Viewer

`kinect_point_cloud.py` (Kinect v2 only) opens a live 3D point cloud window, colored by distance (red = near, blue = far), with a wireframe showing the sensor's field of view — useful for checking sensor placement and framing before an event.
```bash
./run_point_cloud.sh --min-depth 500 --max-depth 4500
```
Left-click drag to rotate, right-click drag to pan, scroll to zoom.
