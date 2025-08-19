import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import subprocess
import time

LIDAR_TOPIC = "/ldlidar_node/scan"
CSV_FILENAME = "reported_data.csv"

class LidarCSVLogger(Node):
    def __init__(self):
        super().__init__("lidar_csv_logger")
        self.subscription = self.create_subscription(
            LaserScan,
            LIDAR_TOPIC,
            self.listener_callback,
            10
        )
        self.scans_written = 0

        # Create CSV header
        with open(CSV_FILENAME, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["scan_id", "angle_deg", "range_m"])

    def listener_callback(self, msg):
        angle = msg.angle_min
        increment = msg.angle_increment
        with open(CSV_FILENAME, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            for i, r in enumerate(msg.ranges):
                angle_deg = angle * 180.0 / 3.14159
                writer.writerow([self.scans_written, round(angle_deg, 2), round(r, 4)])
                angle += increment
        self.get_logger().info(f"Saved scan {self.scans_written} with {len(msg.ranges)} points.")
        self.scans_written += 1


def start_lidar_node():
    print("Starting LiDAR node...")
    subprocess.Popen(["ros2", "launch", "ldlidar_node", "ldlidar.launch.py"])
    time.sleep(2)  # wait for node to start

    # Try lifecycle configuration + activation
    try:
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "configure"], check=True)
        subprocess.run(["ros2", "lifecycle", "set", "/ldlidar_node", "activate"], check=True)
        print("Lifecycle node configured and activated.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to configure/activate lifecycle node: {e}")


def main(args=None):
    rclpy.init(args=args)
    start_lidar_node()
    node = LidarCSVLogger()
    print("Listening for scans...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
