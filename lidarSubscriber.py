import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import subprocess
import time

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

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
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
