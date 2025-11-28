import pymysql
import pandas as pd
import json
import numpy as np

db = pymysql.connect(
    host='localhost',
    user='root',
    password='123',
    database='rosdb',
    charset='utf8mb4'
)
cursor = db.cursor()

# 2 MySQL에서 데이터 가져오기
sql = "SELECT ranges, action FROM lidardata"
cursor.execute(sql)
rows = cursor.fetchall()

# 데이터프레임 생성
data_list = []
for ranges_json, action in rows:
    ranges = json.loads(ranges_json)
    data_list.append(ranges + [action])

# 컬럼 이름 만들기
columns = [f"dist_{i}" for i in range(360)] + ["action"]

df = pd.DataFrame(data_list, columns=columns)
# csv 변환
df.to_csv("lidar_range.csv", index = False)

print(df.head())
print(df.shape)  


cursor.close()
db.close()
