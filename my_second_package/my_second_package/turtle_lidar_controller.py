import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

SAFE_DIST = 0.5  # 안전거리

class TurtleLidarController(Node):
    def __init__(self):
        super().__init__('turtle_lidar_controller')
        
        # /scan 구독
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # /turtle1/cmd_vel 발행
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.get_logger().info("TurtleLidarController started.")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        
        # front: -10 ~ +10 degree
        front = np.r_[ranges[-10:], ranges[:10]]
        left  = ranges[80:100]
        right = ranges[260:280]

        front_dist = np.mean(front)
        left_dist  = np.mean(left)
        right_dist = np.mean(right)

        # Twist 메시지 생성
        twist = Twist()
        if front_dist < SAFE_DIST:
            # 장애물 있음 → 회전
            twist.linear.x = 0.0
            twist.angular.z = 1.0 if left_dist > right_dist else -1.0
            action = "turn_left" if left_dist > right_dist else "turn_right"
        else:
            # 직진
            twist.linear.x = 2.0
            twist.angular.z = 0.0
            action = "go_forward"

        # 명령 발행
        self.cmd_pub.publish(twist)

        # 로그 출력
        self.get_logger().info(f"front={front_dist:.2f}, left={left_dist:.2f}, right={right_dist:.2f}, action={action}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleLidarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
