# Kinect v1 to MadMapper OSC Driver

A lightweight Python driver designed to connect an Xbox 360 Kinect (v1) to a MacBook Air M2, parse the depth feed for moving bodies, and send their coordinates via OSC to MadMapper for interactive projection mapping.

## Setup Instructions for Apple Silicon (M2 Mac)

Because this runs on an M2 Mac, we use `libfreenect` which provides the most stable native support for Kinect v1.

### 1. Hardware Requirements
*   **Kinect v1 (Xbox 360)**
*   **Kinect AC Power Adapter:** The USB cable alone does *not* provide enough power. You must use the wall adapter.
*   **High-Bandwidth USB-C Adapter:** Connect the USB-A end of the Kinect to your M2 Mac using a high-quality USB-C hub or adapter.

### 2. Install System Dependencies
Open your Terminal and install Homebrew if you haven't already:
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Then, install the `libfreenect` library, which contains the drivers to communicate with the hardware:
```bash
brew install libfreenect
```

### 3. Setup Python Environment
Navigate to this folder in your terminal, then create and activate a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
```

Install the required Python packages:
```bash
pip install -r requirements.txt
```
*Note: Depending on your Python installation, `freenect` python bindings might require building from source if not bundled via brew. If you get `ModuleNotFoundError: No module named 'freenect'`, you can install the python wrapper via `pip install freenect` or by building the python wrapper from the official `libfreenect` GitHub repo.*

## Running the Driver

1. Ensure MadMapper is open and OSC input is enabled (Preferences -> OSC -> check "Enable OSC Input", note the Port, usually 8000).
2. Connect the Kinect.
3. Run the script:
```bash
python kinect_osc_driver.py --debug
```

The `--debug` flag opens a visual window showing what the Kinect "sees" so you can adjust your physical position.

## How to use in MadMapper
Once the script is running and detecting people, you will see OSC messages arriving in MadMapper's OSC monitor. 
The driver sends coordinates between `0.0` and `1.0`.

*   `/kinect/person1/x`
*   `/kinect/person1/y`

Right click any parameter in MadMapper (e.g. Opacity, X Position, Rotation), select **Add OSC Control**, and assign it to the incoming `/kinect/person1/x` address.
