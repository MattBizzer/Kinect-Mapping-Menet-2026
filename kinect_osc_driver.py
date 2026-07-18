import cv2
import numpy as np
import freenect
from pythonosc.udp_client import SimpleUDPClient
import argparse
import time

# --- Configuration ---
# MadMapper OSC Configuration
OSC_IP = "127.0.0.1"
OSC_PORT = 8000

# Depth Thresholds (Adjust these based on your physical space)
# Kinect v1 raw depth values: 0 (closest) to 2047 (furthest)
MIN_DEPTH = 400   # Closest distance to track
MAX_DEPTH = 900   # Furthest distance to track
MIN_BLOB_AREA = 3000 # Minimum size of a "person" to ignore noise

def get_depth():
    """
    Retrieves the depth map from the Kinect v1.
    """
    array, _ = freenect.sync_get_depth()
    # Convert to numpy array
    return np.array(array)

def main():
    parser = argparse.ArgumentParser(description="Kinect v1 to MadMapper OSC Driver")
    parser.add_argument("--ip", default=OSC_IP, help="The IP of the MadMapper machine")
    parser.add_argument("--port", type=int, default=OSC_PORT, help="The OSC port MadMapper is listening on")
    parser.add_argument("--debug", action="store_true", help="Show the OpenCV debug window")
    args = parser.parse_args()

    # Initialize OSC Client
    client = SimpleUDPClient(args.ip, args.port)
    print(f"📡 Sending OSC data to {args.ip}:{args.port}")
    print("🎥 Starting Kinect capture... Press Ctrl+C to stop.")
    
    if args.debug:
        print("🐛 Debug mode enabled. A window will open showing the depth feed.")

    try:
        while True:
            # 1. Get depth frame
            depth_map = get_depth()
            
            if depth_map is None:
                print("Failed to get depth map from Kinect. Is it plugged in?")
                time.sleep(1)
                continue

            # 2. Thresholding: Create a mask of objects within our depth range
            # Set pixels within range to 255 (white), everything else to 0 (black)
            mask = np.zeros_like(depth_map, dtype=np.uint8)
            mask[(depth_map > MIN_DEPTH) & (depth_map < MAX_DEPTH)] = 255

            # Apply morphological operations to clean up noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # 3. Blob Tracking: Find contours of the objects
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Sort contours by area (largest first) to find the primary targets
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            person_count = 0
            # Track up to 3 people/blobs for this example
            for cnt in contours[:3]:
                area = cv2.contourArea(cnt)
                if area > MIN_BLOB_AREA:
                    person_count += 1
                    
                    # Calculate center (centroid) of the blob
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # Normalize coordinates to 0.0 - 1.0 range for MadMapper
                        # Kinect v1 depth resolution is usually 640x480
                        norm_x = cx / 640.0
                        norm_y = cy / 480.0

                        # 4. Send OSC Messages
                        # Sends paths like: /kinect/person1/x
                        client.send_message(f"/kinect/person{person_count}/x", norm_x)
                        client.send_message(f"/kinect/person{person_count}/y", norm_y)
                        # Optional: Send raw depth of center point
                        raw_depth = float(depth_map[cy, cx])
                        client.send_message(f"/kinect/person{person_count}/depth", raw_depth)

                        if args.debug:
                            # Draw bounding box and center on the mask for visual feedback
                            x, y, w, h = cv2.boundingRect(cnt)
                            cv2.rectangle(mask, (x, y), (x+w, y+h), (128), 2)
                            cv2.circle(mask, (cx, cy), 5, (255), -1)

            if args.debug:
                cv2.imshow("Kinect Depth Tracking (Debug)", mask)
                # Wait 10ms for a key event, allows cv2 to render the window
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\n🛑 Stopping Kinect driver.")
    finally:
        if args.debug:
            cv2.destroyAllWindows()
            
if __name__ == "__main__":
    main()
