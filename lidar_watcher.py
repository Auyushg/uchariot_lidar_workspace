import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
import numpy as np

PROXIMITY_THRESH = 0.2   # meters

class StickyProximityGUI(Node):
    def __init__(self):
        super().__init__('sticky_proximity_gui')
        self.sub = self.create_subscription(
            LaserScan, '/ldlidar_node/scan', self.scan_callback, 10)

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots(subplot_kw={'projection':'polar'})
        self.ax.set_theta_direction(1)     # angles increase CCW
        self.ax.set_theta_zero_location('N')  # place 0° at the top (where 90° was)
        self.ax.set_ylim(0, 2)
        self.scatter_all  = None
        self.scatter_flag = None
        self.flagged = None   # will be a boolean array of length N beams
        tick_degs = np.arange(0, 360, 45)
        tick_locs = np.deg2rad(tick_degs)
        self.ax.set_xticks(tick_locs)

        # build labels but swap 90° and 270°
        labels = []
        for d in tick_degs:
            if d == 90:
                labels.append("270°")
            elif d == 270:
                labels.append("90°")
            else:
                labels.append(f"{d}°")

        self.ax.set_xticklabels(labels)
    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        N      = ranges.size

        # generate beam angles only once (or if N changes)
        if self.flagged is None or self.flagged.size != N:
            self.flagged = np.zeros(N, dtype=bool)
            self.angles = msg.angle_min + np.arange(N)*msg.angle_increment

        # update sticky flags
        valid = (ranges > 0.0) & np.isfinite(ranges)
        below = valid & (ranges < PROXIMITY_THRESH)
        # once flagged = True, stay until reading >= threshold
        self.flagged = np.where(below, True, valid & (ranges >= PROXIMITY_THRESH) & False | self.flagged)

        # prepare data for plotting
        radii = np.where(valid, ranges, np.nan)
        flagged_theta = self.angles[self.flagged]
        flagged_r     = radii[self.flagged]

        # draw
        if self.scatter_all is None:
            self.scatter_all  = self.ax.scatter(self.angles,    radii,       c='blue', s=10)
            self.scatter_flag = self.ax.scatter(flagged_theta, flagged_r, c='red',  s=20)
        else:
            self.scatter_all.set_offsets(np.c_[self.angles, radii])
            self.scatter_flag.set_offsets(np.c_[flagged_theta, flagged_r])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = StickyProximityGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
