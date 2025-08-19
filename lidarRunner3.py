import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import subprocess
import time
import signal
import sys
from collections import deque
import math

lidar_proc = None
node = None

# keep a sliding window (you had 50)
scan_window = deque(maxlen=50)

def cleanup():
    print("\n[INFO] Cleaning up...")
    try:
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "deactivate"], timeout=2)
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "shutdown"], timeout=2)
    except Exception as e:
        print(f"[WARN] Lifecycle transition failed: {e}")

    if lidar_proc and lidar_proc.poll() is None:
        print("[INFO] Terminating lidar launch process...")
        lidar_proc.terminate()
        try:
            lidar_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            lidar_proc.kill()

    if node is not None:
        try:
            node.destroy_node()
        except Exception:
            pass

    if rclpy.ok():
        rclpy.shutdown()

    print("[INFO] Cleanup complete.")
    sys.exit(0)

def signal_handler(sig, frame):
    print("[INFO] Ctrl+C detected.")
    cleanup()

signal.signal(signal.SIGINT, signal_handler)

def start_lidar_node():
    global lidar_proc
    print("[INFO] Launching ldlidar_node...")
    lidar_proc = subprocess.Popen(["ros2", "launch", "ldlidar_node", "ldlidar.launch.py"])
    time.sleep(2.0)  # give it time to come up

    try:
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "configure"], check=True)
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "activate"], check=True)
        print("[INFO] ldlidar_node activated.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Lifecycle setup failed: {e}")
        cleanup()

class SlidingWindowSubscriber(Node):
    def __init__(self):
        super().__init__('sliding_window_subscriber')
        self.scan_count = 0
        self.sub = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: LaserScan):
        # angles in radians (LiDAR native)
        angles_rad = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        ranges = list(msg.ranges)

        # Convert angles and prepare tuples.
        # We'll store both lidar_deg (device-native) and plot_deg (for Matplotlib polar).
        scan = []
        for ang_rad, r in zip(angles_rad, ranges):
            # device native degree, 0..360
            lidar_deg = (ang_rad * 180.0 / math.pi) % 360.0

            # convert to Matplotlib polar coords: 0 deg at North, CCW positive
            plot_theta_rad = (math.pi/2 + ang_rad) % (2*math.pi)
            plot_deg = (plot_theta_rad * 180.0 / math.pi) % 360.0

            # store range as float or None
            if r is None or (isinstance(r, float) and (not math.isfinite(r) or r <= 0.0)):
                rng_val = None
            else:
                rng_val = float(r)

            # Save a tuple with both angle representations and range in meters
            # order: (lidar_deg, plot_deg, range_m)
            scan.append((lidar_deg, plot_deg, rng_val))

        # Append to sliding window
        scan_window.append(scan)
        self.scan_count += 1
        self.get_logger().info(f"Captured scan #{self.scan_count}, window size={len(scan_window)}")

        # Rewrite entire window to CSV (columns: scan_index, lidar_deg, plot_deg, range_m)
        try:
            with open('reported_data.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['scan_index', 'lidar_deg', 'plot_deg', 'range_m'])
                # enumerate over window: 0 is oldest, up to len-1 latest
                for idx, sc in enumerate(scan_window):
                    for lidar_deg, plot_deg, r in sc:
                        if r is None:
                            r_str = ''
                        else:
                            r_str = f"{r:.3f}"
                        writer.writerow([idx, f"{lidar_deg:.2f}", f"{plot_deg:.2f}", r_str])
            self.get_logger().info("reported_data.csv updated")
        except Exception as e:
            self.get_logger().error(f"Failed writing CSV: {e}")

def main(args=None):
    global node
    rclpy.init(args=args)
    start_lidar_node()
    node = SlidingWindowSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt caught.")
    finally:
        cleanup()

if __name__ == '__main__':
    main()
