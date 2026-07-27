import sys
import time
import argparse
import numpy as np
import open3d as o3d


def jet_colormap(t):
    """Vectorized jet colormap. t in [0, 1] -> Nx3 RGB, 0=blue, 1=red."""
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(np.minimum(4 * t - 1.5, -4 * t + 4.5), 0, 1)
    g = np.clip(np.minimum(4 * t - 0.5, -4 * t + 3.5), 0, 1)
    b = np.clip(np.minimum(4 * t + 0.5, -4 * t + 2.5), 0, 1)
    return np.stack((r, g, b), axis=-1)


def build_frustum_lineset(fx, fy, cx, cy, width, height, far_z):
    """Red wireframe pyramid from the sensor origin out to the far viewing plane."""
    pixel_corners = [(0, 0), (width - 1, 0), (width - 1, height - 1), (0, height - 1)]
    far_corners = [
        (-(col - cx) * far_z / fx, (row - cy) * far_z / fy, far_z)
        for col, row in pixel_corners
    ]
    points = [(0.0, 0.0, 0.0)] + far_corners
    lines = [(0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (2, 3), (3, 4), (4, 1)]

    frustum = o3d.geometry.LineSet()
    frustum.points = o3d.utility.Vector3dVector(points)
    frustum.lines = o3d.utility.Vector2iVector(lines)
    frustum.colors = o3d.utility.Vector3dVector([[1.0, 0.0, 0.0]] * len(lines))
    return frustum


class MacKinectV2Capture:
    """Kinect v2 depth capture on macOS/Linux via libfreenect2."""
    def __init__(self):
        from pylibfreenect2 import Freenect2, SyncMultiFrameListener
        from pylibfreenect2 import FrameType, Registration, Frame
        from pylibfreenect2 import OpenGLPacketPipeline, CpuPacketPipeline

        try:
            pipeline = OpenGLPacketPipeline()
            print("🟢 Using OpenGL GPU accelerated pipeline.")
        except Exception as e:
            print(f"⚠️ OpenGL acceleration failed ({e}). Falling back to CPU pipeline.")
            pipeline = CpuPacketPipeline()

        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("❌ Error: No Kinect v2 devices connected.")
            sys.exit(1)

        serial = fn.getDeviceSerialNumber(0)
        self.device = fn.openDevice(serial, pipeline=pipeline)

        # Color frames are still requested because libfreenect2's registration
        # step needs both to produce the undistorted depth frame, even though
        # we don't use the color pixels themselves (points are colored by depth).
        self.listener = SyncMultiFrameListener(FrameType.Color | FrameType.Depth)
        self.device.setColorFrameListener(self.listener)
        self.device.setIrAndDepthFrameListener(self.listener)
        self.device.start()
        print(f"🎬 Started Kinect v2 (Serial: {serial})")

        ir_params = self.device.getIrCameraParams()
        color_params = self.device.getColorCameraParams()
        self.fx, self.fy = ir_params.fx, ir_params.fy
        self.cx, self.cy = ir_params.cx, ir_params.cy

        self.registration = Registration(ir_params, color_params)
        self.undistorted = Frame(512, 424, 4)
        self.registered = Frame(512, 424, 4)
        self.width, self.height = 512, 424

    def get_intrinsics(self):
        return self.fx, self.fy, self.cx, self.cy

    def get_depth(self):
        # waitForNewFrame blocks until a frame is ready, so this naturally paces the loop.
        frames = self.listener.waitForNewFrame()
        color = frames["color"]
        depth = frames["depth"]
        self.registration.apply(color, depth, self.undistorted, self.registered)
        d_arr = np.copy(self.undistorted.asarray(np.float32))
        self.listener.release(frames)
        return d_arr

    def close(self):
        self.device.stop()
        self.device.close()


class WindowsKinectV2Capture:
    """Kinect v2 depth capture on Windows via the official Kinect for Windows SDK 2.0."""
    def __init__(self):
        try:
            from pykinect2 import PyKinectV2, PyKinectRuntime
        except ImportError:
            print("\n❌ Error: The 'pykinect2' module is not installed or cannot be found.")
            print("Install the official Kinect for Windows SDK 2.0, then:")
            print("  pip install pykinect2 comtypes\n")
            sys.exit(1)

        print("🟢 Initializing Kinect v2 (Xbox One) via Kinect for Windows SDK...")
        try:
            self.runtime = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
        except Exception as e:
            print(f"❌ Error initializing Kinect v2: {e}")
            print("Verify the Kinect for Windows SDK 2.0 is installed and the sensor is connected/powered.")
            sys.exit(1)
        print("🎬 Started Kinect v2")

        self.width = self.runtime.depth_frame_desc.Width
        self.height = self.runtime.depth_frame_desc.Height

        intrinsics = self.runtime._mapper.GetDepthCameraIntrinsics()
        self.fx = intrinsics.FocalLengthX
        self.fy = intrinsics.FocalLengthY
        self.cx = intrinsics.PrincipalPointX
        self.cy = intrinsics.PrincipalPointY

    def get_intrinsics(self):
        return self.fx, self.fy, self.cx, self.cy

    def get_depth(self):
        # Non-blocking - returns None between frames, unlike the macOS listener.
        if not self.runtime.has_new_depth_frame():
            return None
        frame = self.runtime.get_last_depth_frame()
        return frame.reshape((self.height, self.width)).astype(np.float32)

    def close(self):
        self.runtime.close()


def main():
    parser = argparse.ArgumentParser(description="Kinect v2 Real-Time 3D Point Cloud Visualizer")
    parser.add_argument("--min-depth", type=float, default=500.0, help="Minimum depth to display (in mm, default: 500)")
    parser.add_argument("--max-depth", type=float, default=3000.0, help="Maximum depth to display (in mm, default: 3000)")
    parser.add_argument("--skip-frames", type=int, default=1, help="Process every Nth frame to optimize rendering (default: 1)")
    args = parser.parse_args()

    capture = WindowsKinectV2Capture() if sys.platform == "win32" else MacKinectV2Capture()
    fx, fy, cx, cy = capture.get_intrinsics()
    width, height = capture.width, capture.height

    # Pre-generate the 2D grid index matrix for vectorized math
    rows, cols = np.meshgrid(np.arange(height), np.arange(width), indexing='ij')

    # Setup Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Kinect v2 3D Point Cloud", width=1024, height=768)

    # Create empty geometry
    pcd = o3d.geometry.PointCloud()

    # Add dummy point to initialize geometry in the visualizer
    pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    pcd.colors = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    vis.add_geometry(pcd)

    # Sensor field-of-view wireframe, like the Kinect Studio 3D view
    frustum = build_frustum_lineset(fx, fy, cx, cy, width, height, args.max_depth / 1000.0)
    vis.add_geometry(frustum)

    # Set up view control options
    view_control = vis.get_view_control()
    # Looking down the negative Z-axis initially
    view_control.set_front([0.0, 0.0, -1.0])
    view_control.set_up([0.0, -1.0, 0.0]) # Coordinate system has +Y pointing down

    # Render options (e.g. background color, point size)
    render_option = vis.get_render_option()
    render_option.background_color = np.array([0.1, 0.1, 0.1])
    render_option.point_size = 2.0

    print("\n👉 Controls in 3D Viewer:")
    print("   - Left Click + Drag: Rotate")
    print("   - Right Click + Drag: Translate (Pan)")
    print("   - Scroll Wheel: Zoom")
    print("   - Press ESC or close the window to exit\n")

    frame_count = 0
    fps_start_time = time.time()
    fps_frames = 0
    view_initialized = False
    last_point_count = 0

    try:
        while vis.poll_events():
            d_arr = capture.get_depth()
            if d_arr is None:
                time.sleep(0.001)
                continue

            frame_count += 1
            if frame_count % args.skip_frames != 0:
                continue

            # Filter points based on depth parameters and valid values
            valid = (d_arr > args.min_depth) & (d_arr < args.max_depth)
            z_vals = d_arr[valid]

            if len(z_vals) > 0:
                # Vectorized calculations for 3D coordinates (X, Y, Z) in meters.
                # X is negated so left/right matches reality instead of the sensor's
                # raw mirror-image view.
                x_vals = -(cols[valid] - cx) * z_vals / fx / 1000.0
                y_vals = (rows[valid] - cy) * z_vals / fy / 1000.0
                z_vals = z_vals / 1000.0

                points = np.stack((x_vals, y_vals, z_vals), axis=-1)

                # Color by distance (blue=far, red=near), like Kinect Studio's 3D view
                depth_t = 1.0 - (z_vals - args.min_depth / 1000.0) / (
                    (args.max_depth - args.min_depth) / 1000.0
                )
                colors = jet_colormap(depth_t)

                # Update the visualizer geometry
                pcd.points = o3d.utility.Vector3dVector(points)
                pcd.colors = o3d.utility.Vector3dVector(colors)
                vis.update_geometry(pcd)
                last_point_count = len(z_vals)

                # Frame the camera on the real point cloud once data arrives. Fitting via
                # reset_view_point() would size the zoom to include the (much larger)
                # frustum wireframe too, shrinking the subject to a speck in the middle.
                # A slight angle off the sensor's own axis also gives real depth
                # perspective, instead of the flat, straight-on view down that axis.
                if not view_initialized:
                    view_control.set_lookat(pcd.get_center())
                    view_control.set_front([0.15, -0.12, -0.98])
                    view_control.set_up([0.0, -1.0, 0.0])
                    view_control.set_zoom(0.5)
                    # Default FOV (60 deg) skews rectangular surfaces into trapezoids
                    # at this range. Narrow it to cut that perspective distortion.
                    view_control.change_field_of_view(step=-25)
                    view_initialized = True

            # Render frame updates
            vis.update_renderer()

            # FPS counter
            fps_frames += 1
            current_time = time.time()
            if current_time - fps_start_time >= 1.0:
                fps = fps_frames / (current_time - fps_start_time)
                # Print status
                sys.stdout.write(f"\rRender FPS: {fps:.2f} | Points: {last_point_count:,}   ")
                sys.stdout.flush()
                fps_frames = 0
                fps_start_time = current_time

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        print("\nClosing streams and visualizer...")
        vis.destroy_window()
        capture.close()
        print("Bye!")

if __name__ == "__main__":
    main()
