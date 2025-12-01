import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import csv

class LidarAnalyze(Node):
    def __init__(self):
        super().__init__('lidar_analyzer')
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, 10)
        self.data = []  # CSV에 저장할 데이터

    def listener_callback(self, msg):
        # 1. ROS data to Numpy
        ranges = np.array(msg.ranges)

        # 2. preprocessing :inf 데이터 처리
        ranges = np.where(ranges == float('inf'), 3.5, ranges)

        # 3. 각 방향 체크
        front_ranges = np.concatenate((ranges[:30], ranges[-30:]))
        left_ranges = ranges[80:100]
        right_ranges = ranges[260:280]
        back_ranges = ranges[160:200]

        safe_dist = 0.3
        danger_flags = [
            np.min(front_ranges) < safe_dist,
            np.min(left_ranges) < safe_dist,
            np.min(right_ranges) < safe_dist,
            np.min(back_ranges) < safe_dist
        ]

        # 동시에 2개 이상 위험이면 제외
        if sum(danger_flags) > 1:
            return

        # state 설정 (5가지)
        state_map = ['front_danger', 'left_danger', 'right_danger', 'back_danger']
        if sum(danger_flags) == 0:
            state = 'safe'
        else:
            state = state_map[danger_flags.index(True)]

        # CSV용 row 생성 (360개 거리 + state)
        row = list(ranges) + [state]
        self.data.append(row)

        # 로그
        if state != 'safe':
            self.get_logger().warn(f"{state} min distance: {np.min(ranges):.3f} m")
        else:
            self.get_logger().info("safe ranges...")

    def save_csv(self, filename="lidar_data.csv"):
        if not self.data:
            return
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # header: angle_0 ~ angle_359 + state
            header = [f"angle_{i}" for i in range(len(self.data[0])-1)] + ['state']
            writer.writerow(header)
            writer.writerows(self.data)
        print(f"CSV saved: {filename}")

def main():
    rclpy.init()
    node = LidarAnalyze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C pressed. Saving CSV...")
        node.save_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
