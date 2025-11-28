import roslibpy
import numpy as np
import pymysql
import json

# -----------------------------------
# MySQL 연결
# -----------------------------------
db = pymysql.connect(
    host='localhost',
    user='root',
    password='123',
    database='rosdb',
    charset='utf8mb4'
)

cursor = db.cursor()


# ROS 연결
client = roslibpy.Ros(host='localhost', port=9090)
client.run()
print('Connected to ROS!')

cmd_vel_pub = roslibpy.Topic(
            client,
            '/turtle1/cmd_vel',
            'geometry_msgs/msg/Twist'
        )
listener = roslibpy.Topic(client,
                            '/lidar_data_mock',
                            'sensor_msgs/msg/LaserScan')


# 콜백 함수 (분석 + MySQL INSERT)
def callback(message):
    ranges = np.array(message["ranges"])

    front = np.r_[ranges[350:360], ranges[0:10]]
    left  = ranges[80:100]
    right = ranges[260:280]

    front_dist = np.mean(front)
    left_dist  = np.mean(left)
    right_dist = np.mean(right)

    safe_dist = 0.5

    if front_dist < safe_dist:
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
    else:
        action = "go_forward"
        cmd_vel_pub.publish({
            'linear': {'x': 0.5},
            'angular': {'z': 0.0}
        })

    print("front:", round(front_dist, 2))
    print("left :", round(left_dist, 2))
    print("right:", round(right_dist, 2))
    print("action:", action)


    # MySQL INSERT 저장
    sql = """
        INSERT INTO lidardata (ranges, action)
        VALUES (%s, %s)
    """
    cursor.execute(sql, (json.dumps(message["ranges"]), action))
    db.commit()



# 실행 루프
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
    print('Disconnected from ROS & MySQL.')
