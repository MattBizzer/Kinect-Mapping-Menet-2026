import cv2
import numpy as np
from pythonosc.udp_client import SimpleUDPClient
import argparse
import time
import sys

# --- Configuration Defaults ---
OSC_IP = "127.0.0.1"
OSC_PORT = 8000
MIN_BLOB_AREA = 3000

class KinectV1Camera:
    def __init__(self):
        try:
            import freenect
            self.freenect = freenect
        except ImportError:
            print("\n❌ Error: The 'freenect' module is not installed or cannot be found.")
            print("Please make sure you have run './build_drivers.sh' and activated the virtual environment:")
            print("  source venv/bin/activate\n")
            sys.exit(1)
        self.width = 640
        self.height = 480
        self.default_min_depth = 400
        self.default_max_depth = 900

    def start(self):
        print("🟢 Initializing Kinect v1 (Xbox 360)...")
        # Test connection
        try:
            self.freenect.sync_get_depth()
        except Exception as e:
            print(f"❌ Error communicating with Kinect v1: {e}")
            print("Verify the Kinect is powered via its wall adapter and connected to the USB port.")
            sys.exit(1)

    def get_depth(self):
        try:
            array, _ = self.freenect.sync_get_depth()
            if array is None:
                return None
            return np.array(array)
        except TypeError:
            # Occasional sync/driver issues
            return None

    def stop(self):
        try:
            self.freenect.sync_stop()
        except:
            pass

class KinectV2Camera:
    def __init__(self):
        try:
            from pylibfreenect2 import Freenect2, SyncMultiFrameListener, FrameType
            self.freenect2 = Freenect2()
            self.FrameType = FrameType
            self.SyncMultiFrameListener = SyncMultiFrameListener
        except ImportError:
            print("\n❌ Error: The 'pylibfreenect2' module is not installed or cannot be found.")
            print("Please make sure you have run './build_drivers.sh' and activated the virtual environment:")
            print("  source venv/bin/activate\n")
            sys.exit(1)
        self.width = 512
        self.height = 424
        self.default_min_depth = 500    # 500mm = 0.5m
        self.default_max_depth = 2500   # 2500mm = 2.5m
        self.device = None
        self.listener = None

    def start(self):
        print("🟢 Initializing Kinect v2 (Xbox One)...")
        num_devices = self.freenect2.enumerateDevices()
        if num_devices == 0:
            print("❌ Error: No Kinect v2 devices connected.")
            print("Verify the Kinect v2 is powered and connected to a USB 3.0/USB4 port.")
            sys.exit(1)

        serial = self.freenect2.getDeviceSerialNumber(0)
        self.device = self.freenect2.openDevice(serial)

        self.listener = self.SyncMultiFrameListener(self.FrameType.Depth)
        self.device.setIrAndDepthFrameListener(self.listener)
        self.device.start()

    def get_depth(self):
        if self.listener is None:
            return None
        # waitForNewFrame block-waits until a frame is ready
        frames = self.listener.waitForNewFrame()
        depth_frame = frames["depth"]
        depth_data = depth_frame.asarray()

        # Make a copy of the depth array before releasing frames
        depth_copy = np.copy(depth_data)
        self.listener.release(frames)

        # Kinect v2 can report NaNs for out-of-range or shadowed pixels
        depth_copy = np.nan_to_num(depth_copy, nan=0.0)
        return depth_copy

    def stop(self):
        if self.device is not None:
            self.device.stop()
            self.device.close()

class KinectV2WindowsCamera:
    """Kinect v2 via the official Kinect for Windows SDK 2.0 (pykinect2), used
    instead of libfreenect2 on Windows where the official driver is available."""
    def __init__(self):
        try:
            from pykinect2 import PyKinectV2, PyKinectRuntime
            self.PyKinectV2 = PyKinectV2
            self.PyKinectRuntime = PyKinectRuntime
        except ImportError:
            print("\n❌ Error: The 'pykinect2' module is not installed or cannot be found.")
            print("Install the official Kinect for Windows SDK 2.0, then:")
            print("  pip install pykinect2 comtypes\n")
            sys.exit(1)
        self.width = 512
        self.height = 424
        self.default_min_depth = 500    # 500mm = 0.5m
        self.default_max_depth = 2500   # 2500mm = 2.5m
        self.runtime = None

    def start(self):
        print("🟢 Initializing Kinect v2 (Xbox One) via Kinect for Windows SDK...")
        try:
            self.runtime = self.PyKinectRuntime.PyKinectRuntime(self.PyKinectV2.FrameSourceTypes_Depth)
        except Exception as e:
            print(f"❌ Error initializing Kinect v2: {e}")
            print("Verify the Kinect for Windows SDK 2.0 is installed and the sensor is connected/powered.")
            sys.exit(1)

    def get_depth(self):
        if self.runtime is None or not self.runtime.has_new_depth_frame():
            return None
        # Flat uint16 array (width*height) in millimeters -> reshape to (rows, cols)
        frame = self.runtime.get_last_depth_frame()
        return frame.reshape((self.height, self.width)).astype(np.float32)

    def stop(self):
        if self.runtime is not None:
            self.runtime.close()

def calibrate_wall(camera, num_frames):
    """Average depth over a settle period + several frames to get a stable per-pixel
    baseline of the empty surface, so touches can be detected as departures from it."""
    print(f"📏 Calibrating wall surface — keep the area clear of people for a moment...")
    for _ in range(10):
        camera.get_depth()
        time.sleep(0.05)

    accum = None
    valid_counts = None
    captured = 0
    while captured < num_frames:
        depth_map = camera.get_depth()
        if depth_map is None:
            continue
        if accum is None:
            accum = np.zeros_like(depth_map, dtype=np.float64)
            valid_counts = np.zeros_like(depth_map, dtype=np.float64)
        valid = depth_map > 0
        accum[valid] += depth_map[valid]
        valid_counts[valid] += 1
        captured += 1

    background = np.zeros_like(accum)
    has_data = valid_counts > 0
    background[has_data] = accum[has_data] / valid_counts[has_data]
    print("✅ Wall calibration complete.")
    return background


def run_touch_mode(camera, client, args):
    background = calibrate_wall(camera, args.calibration_frames)
    active_touches = 0

    print("🎥 Touch tracking started... Press Ctrl+C to stop.")
    if args.debug:
        print("🐛 Debug mode enabled. A window will open showing the touch mask.")

    while True:
        depth_map = camera.get_depth()
        if depth_map is None:
            time.sleep(0.01)
            continue

        # Positive where something sits closer to the sensor than the calibrated
        # wall - i.e. in front of the surface, by roughly a hand's depth or less.
        offset = background - depth_map
        touch_mask = np.zeros_like(depth_map, dtype=np.uint8)
        touch_mask[
            (depth_map > 0)
            & (background > 0)
            & (offset > args.touch_min_offset)
            & (offset < args.touch_max_offset)
        ] = 255

        kernel = np.ones((5, 5), np.uint8)
        touch_mask = cv2.erode(touch_mask, kernel, iterations=1)
        touch_mask = cv2.dilate(touch_mask, kernel, iterations=2)

        contours, _ = cv2.findContours(touch_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        touch_count = 0
        for cnt in contours[:args.max_touches]:
            area = cv2.contourArea(cnt)
            if area > args.min_area:
                touch_count += 1

                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    norm_x = cx / float(camera.width)
                    norm_y = cy / float(camera.height)

                    client.send_message(f"/kinect/touch{touch_count}/x", norm_x)
                    client.send_message(f"/kinect/touch{touch_count}/y", norm_y)
                    client.send_message(f"/kinect/touch{touch_count}/active", 1)

                    if args.debug:
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(touch_mask, (x, y), (x + w, y + h), (128), 2)
                        cv2.circle(touch_mask, (cx, cy), 5, (255), -1)

        # Tell MadMapper about touches that just ended
        for i in range(touch_count + 1, active_touches + 1):
            client.send_message(f"/kinect/touch{i}/active", 0)
        active_touches = touch_count

        if args.debug:
            cv2.imshow("Kinect Touch Tracking (Debug)", touch_mask)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break


def main():
    parser = argparse.ArgumentParser(description="Kinect to MadMapper OSC Driver")
    parser.add_argument("--camera", choices=["v1", "v2"], default="v1", help="Select Kinect camera version (v1 = Xbox 360, v2 = Xbox One)")
    parser.add_argument("--mode", choices=["track", "touch"], default="track", help="track = person centroid tracking (default), touch = detect contact with a calibrated surface")
    parser.add_argument("--ip", default=OSC_IP, help="The IP of the MadMapper machine")
    parser.add_argument("--port", type=int, default=OSC_PORT, help="The OSC port MadMapper is listening on")
    parser.add_argument("--min-depth", type=float, help="Closest distance/value to track (defaults: v1=400, v2=500mm)")
    parser.add_argument("--max-depth", type=float, help="Furthest distance/value to track (defaults: v1=900, v2=2500mm)")
    parser.add_argument("--min-area", type=int, default=MIN_BLOB_AREA, help="Minimum blob area to track")
    parser.add_argument("--debug", action="store_true", help="Show the OpenCV debug window")
    parser.add_argument("--touch-min-offset", type=float, default=20.0, help="[touch mode] Minimum distance (mm) in front of the wall to count as a touch, filters out sensor noise at the surface (default: 20)")
    parser.add_argument("--touch-max-offset", type=float, default=150.0, help="[touch mode] Maximum distance (mm) in front of the wall still counted as a touch (default: 150)")
    parser.add_argument("--calibration-frames", type=int, default=30, help="[touch mode] Number of frames averaged to calibrate the empty wall surface (default: 30)")
    parser.add_argument("--max-touches", type=int, default=3, help="[touch mode] Maximum simultaneous touch points to track (default: 3)")
    args = parser.parse_args()

    # Initialize the correct Camera wrapper. On Windows, v2 uses the official
    # Kinect for Windows SDK instead of libfreenect2, which isn't there.
    if args.camera == "v1":
        camera = KinectV1Camera()
    elif sys.platform == "win32":
        camera = KinectV2WindowsCamera()
    else:
        camera = KinectV2Camera()

    # Set depth bounds
    min_depth = args.min_depth if args.min_depth is not None else camera.default_min_depth
    max_depth = args.max_depth if args.max_depth is not None else camera.default_max_depth

    # Initialize OSC Client
    client = SimpleUDPClient(args.ip, args.port)
    print(f"📡 Sending OSC data to {args.ip}:{args.port}")

    try:
        camera.start()

        if args.mode == "touch":
            run_touch_mode(camera, client, args)
            return

        print("🎥 Capture started... Press Ctrl+C to stop.")

        if args.debug:
            print("🐛 Debug mode enabled. A window will open showing the depth feed.")

        while True:
            # 1. Get depth frame
            depth_map = camera.get_depth()

            if depth_map is None:
                time.sleep(0.01)
                continue

            # 2. Thresholding
            mask = np.zeros_like(depth_map, dtype=np.uint8)
            mask[(depth_map > min_depth) & (depth_map < max_depth)] = 255

            # Apply morphological operations to clean up noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # 3. Blob Tracking: Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            person_count = 0
            for cnt in contours[:3]:
                area = cv2.contourArea(cnt)
                if area > args.min_area:
                    person_count += 1

                    # Calculate center
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Normalize coordinates using camera dimensions
                        norm_x = cx / float(camera.width)
                        norm_y = cy / float(camera.height)

                        # 4. Send OSC Messages
                        client.send_message(f"/kinect/person{person_count}/x", norm_x)
                        client.send_message(f"/kinect/person{person_count}/y", norm_y)

                        # Get depth at centroid
                        # Bound check coordinates
                        cy_bound = max(0, min(depth_map.shape[0] - 1, cy))
                        cx_bound = max(0, min(depth_map.shape[1] - 1, cx))
                        raw_depth = float(depth_map[cy_bound, cx_bound])
                        client.send_message(f"/kinect/person{person_count}/depth", raw_depth)

                        if args.debug:
                            # Draw bounding box and center
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(mask, (x, y), (x+w, y+h), (128), 2)
                            cv2.circle(mask, (cx, cy), 5, (255), -1)

            if args.debug:
                cv2.imshow("Kinect Depth Tracking (Debug)", mask)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n🛑 Stopping Kinect driver.")
    finally:
        camera.stop()
        if args.debug:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
