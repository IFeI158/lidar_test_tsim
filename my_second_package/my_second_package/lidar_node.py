#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import random 
import math

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5

AVAILABLE_PATTERNS = [
    "front_wall",
    "front_left_wall",
    "front_right_wall",
    "all_blocked",  
    "no_wall"
]

class MyTopicHandler(Node):
    def __init__(self):
        super().__init__('lidar_node')

        self.publisher = self.create_publisher(
            LaserScan,
            '/lidar_data_mock',
            10
        )

        self.timer = self.create_timer(2.0, self.timer_callback)

    def create_empty_scan(self):
        ranges = [RANGE_MAX for _ in range(NUM_POINTS)]
        intensities = [100.0 for _ in range(NUM_POINTS)]
        scan = {
            "angle_min": math.radians(ANGLE_MIN_DEG),
            "angle_max": math.radians(ANGLE_MAX_DEG),
            "angle_increment": math.radians(ANGLE_INCREMENT_DEG),
            "range_min": RANGE_MIN,
            "range_max": RANGE_MAX,
            "ranges": ranges,
            "intensities": intensities
        }
        return scan

    def make_the_wall(self, ranges, center_deg, width_deg):
        half_width = width_deg // 2
        wall_dist = random.uniform(RANGE_MIN, RANGE_MAX)  # << 랜덤 거리 생성
        for offset in range(-half_width, half_width + 1):
            idx = (center_deg + offset) % NUM_POINTS
            ranges[idx] = wall_dist

    def generate_scan_with_combinations(self):
        scan = self.create_empty_scan()

        pattern = random.choice(AVAILABLE_PATTERNS)
        self.get_logger().info(f"Pattern selected: {pattern}")

        if pattern == "front_wall":
            self.make_the_wall(scan["ranges"], center_deg=0, width_deg=40)

        elif pattern == "front_left_wall":
            self.make_the_wall(scan["ranges"], center_deg=0, width_deg=40)
            self.make_the_wall(scan["ranges"], center_deg=90, width_deg=30)

        elif pattern == "front_right_wall":
            self.make_the_wall(scan["ranges"], center_deg=0, width_deg=40)
            self.make_the_wall(scan["ranges"], center_deg=270, width_deg=30)

        elif pattern == "all_blocked":  
            self.make_the_wall(scan["ranges"], center_deg=0, width_deg=40)
            self.make_the_wall(scan["ranges"], center_deg=90, width_deg=30)
            self.make_the_wall(scan["ranges"], center_deg=270, width_deg=30)

        elif pattern == "no_wall":
            pass

        return scan

    def timer_callback(self):
        scan = self.generate_scan_with_combinations()

        my_msg = LaserScan()
        my_msg.angle_min = scan['angle_min']
        my_msg.angle_max = scan['angle_max']
        my_msg.angle_increment = scan['angle_increment']
        my_msg.range_min = scan['range_min']
        my_msg.range_max = scan['range_max']
        my_msg.ranges = scan['ranges']
        my_msg.intensities = scan['intensities']

        self.publisher.publish(my_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyTopicHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
