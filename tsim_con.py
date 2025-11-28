import roslibpy
import numpy as np
import pymysql
import json

# -----------------------------
# MySQL 연결
# -----------------------------
db = pymysql.connect(
    host='localhost',
    user='root',
    password='123',
    database='rosdb',
    charset='utf8mb4'
)
cursor = db.cursor()

# -----------------------------
# ROS 연결
# -----------------------------
client = roslibpy.Ros(host='localhost', port=9090)
client.run()
print('Connected to ROS!')

cmd_vel_pub = roslibpy.Topic(
    client,
    '/turtle1/cmd_vel',
    'geometry_msgs/msg/Twist'
)

listener = roslibpy.Topic(
    client,
    '/lidar_data_mock',
    'sensor_msgs/msg/LaserScan'
)

# -----------------------------
# 콜백 함수: 벽 감지 + 행동 결정 + DB 저장
# -----------------------------
def callback(message):
    ranges = np.array(message["ranges"])

    # 각도별 구역 나누기
    front = np.r_[ranges[350:360], ranges[0:10]]  # 전방
    left  = ranges[80:100]                         # 좌측
    right = ranges[260:280]                        # 우측

    # 최소 거리 기반 벽 감지
    front_dist = np.min(front)
    left_dist  = np.min(left)
    right_dist = np.min(right)

    safe_dist = 3.4  # 벽 존재 판단 기준

    # -----------------------------
    # 의사결정 로직 (4가지 행동)
    # -----------------------------
    if front_dist < safe_dist and left_dist < safe_dist and right_dist < safe_dist:
        # 3방향 모두 막힘 → 후진
        action = "go_backward"
        cmd_vel_pub.publish({
            'linear': {'x': -0.4},
            'angular': {'z': 0.0}
        })

    elif front_dist < safe_dist:
        # 전방 막힘 → 좌/우 선택 회전
        if left_dist > right_dist:
            action = "turn_left"
            cmd_vel_pub.publish({
                'linear': {'x': 0.0},
                'angular': {'z': 1.0}
            })
        else:
            action = "turn_right"
            cmd_vel_pub.publish({
                'linear': {'x': 0.0},
                'angular': {'z': -1.0}
            })

    elif left_dist < safe_dist:
        # 좌측만 막힘 → 우회전
        action = "turn_right"
        cmd_vel_pub.publish({
            'linear': {'x': 0.0},
            'angular': {'z': -1.0}
        })

    elif right_dist < safe_dist:
        # 우측만 막힘 → 좌회전
        action = "turn_left"
        cmd_vel_pub.publish({
            'linear': {'x': 0.0},
            'angular': {'z': 1.0}
        })

    else:
        # 벽 없음 → 전진
        action = "go_forward"
        cmd_vel_pub.publish({
            'linear': {'x': 0.5},
            'angular': {'z': 0.0}
        })

    # 디버깅 출력
    print("front:", round(front_dist, 2))
    print("left :", round(left_dist, 2))
    print("right:", round(right_dist, 2))
    print("action:", action)

    # MySQL 저장
    sql = """
        INSERT INTO lidardata (ranges, action)
        VALUES (%s, %s)
    """
    cursor.execute(sql, (json.dumps(message["ranges"]), action))
    db.commit()


# -----------------------------
# 실행 루프
# -----------------------------
try:
    listener.subscribe(callback)

    while client.is_connected:
        pass

except KeyboardInterrupt:
    pass

finally:
    listener.unsubscribe()
    client.terminate()
    cursor.close()
    db.close()
    print('Disconnected.')
