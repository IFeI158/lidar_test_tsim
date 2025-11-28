# Turtlesim 을 통한 가상의 라이다 시뮬레이션

- ROS PC
    - Lidar 센서 모의 데이터를 랜덤하게 생성하여 2초마다 Publish하는 토픽 만들기
    
    - 1번 항목 이후부터는 각 파트마다 깃과 깃헙으로 관리해가면서 작업하기
    - 결정된 액션(직진, 좌회전, 우회전 등)을 반영하여 거북이의 주행 명령을 Publish하는 토픽 만들기
    - MySQL 서버에 스키마를 생성하고, lidardata 테이블을 생성하여 ranges[] 데이터를 쌓아올리기. 이때 정수 타입의 id, JSON 타입의 ranges[], datatime 타입의 when, 그리고 주행 액션 action 컬럼으로 구성. 이후 테이블에 데이터가 쌓이는 INSERT 구문까지 수행하기!
    
    ```jsx
    -- 1. 스키마(데이터베이스) 생성
    CREATE DATABASE IF NOT EXISTS rosdb
        DEFAULT CHARACTER SET utf8mb4
        DEFAULT COLLATE utf8mb4_general_ci;
    
    -- 2. 스키마 선택
    USE rosdb;
    
    -- 3. lidardata 테이블 생성
    CREATE TABLE IF NOT EXISTS lidardata (
        id INT AUTO_INCREMENT PRIMARY KEY,
        ranges JSON NOT NULL,               -- LiDAR ranges 배열(JSON)
        `when` DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP, -- 저장 시각
        action VARCHAR(50) NOT NULL         -- 주행 액션(Label)
    );
    	
    ```
    
- REMOTE PC
    - 토픽 받아 주행 액션 결정하기
 
  rosbridge 서버 열기

```jsx
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
