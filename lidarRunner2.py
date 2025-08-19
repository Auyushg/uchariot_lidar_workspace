import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import subprocess
import time
import signal
import sys

lidar_proc = None
node = None

def cleanup():
    print("\n[INFO] Cleaning up...")
    try:
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "deactivate"], timeout=2)
    except Exception as e:
        print(f"[WARN] Deactivate failed: {e}")

    try:
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "shutdown"], timeout=2)
    except Exception as e:
        print(f"[WARN] Shutdown failed: {e}")

    if lidar_proc and lidar_proc.poll() is None:
        print("[INFO] Terminating lidar launch process...")
        lidar_proc.terminate()
        try:
            lidar_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            lidar_proc.kill()

    if node is not None:
        node.destroy_node()

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
    time.sleep(2.0)  # Wait for node to start

    try:
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "configure"], check=True)
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "activate"], check=True)
        print("[INFO] ldlidar_node activated.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Lifecycle setup failed: {e}")
        cleanup()

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        with open('reported_data.csv', mode='w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['angle_deg', 'range_m'])

            angle = msg.angle_min
            for r in msg.ranges:
                angle_deg = angle * 180.0 / 3.1415926
                if not (r == float('inf') or r != r):  # skip inf and NaN
                    csv_writer.writerow([round(angle_deg, 2), round(r, 3)])
                angle += msg.angle_increment

        self.get_logger().info('Wrote latest scan to CSV')

def main(args=None):
    global node
    rclpy.init()
    start_lidar_node()

    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[INFO] KeyboardInterrupt caught.")
    finally:
        cleanup()

if __name__ == '__main__':
    main()
