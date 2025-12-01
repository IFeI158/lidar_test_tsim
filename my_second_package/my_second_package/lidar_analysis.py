#Lidar data preprocessing
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarAnalyze(Node):
    def __init__(self):
        super().__init__('lidar_analyzer')
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.listner_callback, 10)
    
    def listner_callback(self, msg):
        # 1. ROS data to Numpy
        ranges = np.array(msg.ranges)

        # 2. preprocessing :inf data
        ranges = np.where(ranges == float('inf'), 3.5, ranges) #inf= 오류값일때는 3.5 inf 가 아닐때는 ranges

        # 3. check front side
        front_ranges = np.concatenate((ranges[:30], ranges[-30:]))
        left_ranges = ranges[80:100]
        right_ranges = ranges[260:280]
        back_ranges = ranges[160:200]

        # 4. calculate min and mean
        min_fdist = np.min(front_ranges)
        avg_fdist = np.mean(front_ranges)
        min_ldist = np.min(left_ranges)
        avg_ldist = np.mean(left_ranges)
        min_rdist = np.min(right_ranges)
        avg_rdist = np.mean(right_ranges)
        min_bdist = np.min(back_ranges)
        avg_bdist = np.mean(back_ranges)

        safe_dist = 0.3
        if min_fdist < safe_dist:
            self.get_logger().warn(f"front dangerous! {min_fdist}m left!") 
        if min_ldist < safe_dist:
            self.get_logger().warn(f"left dangerous! {min_ldist}m left!") 
        if min_rdist < safe_dist:
            self.get_logger().warn(f"right dangerous! {min_rdist}m left!") 
        if min_bdist < safe_dist:
            self.get_logger().warn(f"back dangerous! {min_bdist}m left!") 
        if min_fdist >= safe_dist and min_ldist >= safe_dist and min_rdist >= safe_dist and min_bdist >= safe_dist:
             self.get_logger().info(f"safe ranges... \n front:{avg_fdist},left:{avg_ldist},right:{avg_rdist}m back:{avg_rdist}m left")

def main() :
    rclpy.init()
    node = LidarAnalyze()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()