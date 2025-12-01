import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

NUM_POINTS = 360
ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
RANGE_MIN = 0.12
RANGE_MAX = 3.5

AVAILABLE_PATTERNS = ["front_wall", "left_wall", "right_wall"]

class LidarSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # 2초마다 Publish
        self.get_logger().info("Lidar Simulator started.")

    def timer_callback(self):
        scan = self.generate_single_scan(random.choice(AVAILABLE_PATTERNS))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = math.radians(ANGLE_MIN_DEG)
        msg.angle_max = math.radians(ANGLE_MAX_DEG)
        msg.angle_increment = math.radians(ANGLE_INCREMENT_DEG)
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX

        # float으로 변환 필수!
        msg.ranges = [float(r) for r in scan['ranges']]
        msg.intensities = [float(i) for i in scan['intensities']]

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published /scan pattern={scan['meta']['pattern']}")

   
    # Scan 데이터 생성
    def create_empty_scan(self):
        ranges = [float(RANGE_MAX) for _ in range(NUM_POINTS)]
        intensities = [100.0 for _ in range(NUM_POINTS)]
        scan = {
            "ranges": ranges,
            "intensities": intensities,
            "meta": {}
        }
        return scan

    def make_the_wall(self, ranges, center_deg, width_deg):
        half_width = width_deg // 2
        for offset in range(-half_width, half_width + 1):
            idx = (center_deg + offset) % NUM_POINTS
            ranges[idx] = 0.4

    def pattern_front_wall(self, scan):
        self.make_the_wall(scan["ranges"], center_deg=0, width_deg=40)

    def pattern_left_wall(self, scan):
        self.make_the_wall(scan["ranges"], center_deg=90, width_deg=30)

    def pattern_right_wall(self, scan):
        self.make_the_wall(scan["ranges"], center_deg=270, width_deg=30)

    def generate_single_scan(self, pattern_name):
        scan = self.create_empty_scan()
        if pattern_name == "front_wall":
            self.pattern_front_wall(scan)
        elif pattern_name == "left_wall":
            self.pattern_left_wall(scan)
        elif pattern_name == "right_wall":
            self.pattern_right_wall(scan)
        scan["meta"]["pattern"] = pattern_name
        return scan


# ROS2 entry point
def main(args=None):
    rclpy.init(args=args)
    node = LidarSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
