#!/usr/bin/env python
#docker fastlio2运行
#@pbj
#2025.03.25
import rosbag
import csv
from sensor_msgs.msg import NavSatFix  # 根据你的消息类型调整导入
import yaml

# 从YAML文件加载数据
with open("./config.yaml", "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

bagpath = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+".bag"
csv_file = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"+str(data['bag_name'])+".csv"

# 打开bag文件
bag = rosbag.Bag(bagpath)  # 替换为你的bag文件路径
# bag = rosbag.Bag('/home/mars_ugv/docker_ws/data/2025-03-25/2025-03-25-20-39-22.bag')  # 替换为你的bag文件路径

# 打开CSV文件以写入模式
csv_file = open(csv_file, 'w')
# csv_file = open('/home/mars_ugv/docker_ws/data/2025-03-25/analysis/2025-03-25-20-39-22.csv', 'w')
csv_writer = csv.writer(csv_file)

# 写入CSV文件的标题行，包括原始时间戳
csv_writer.writerow(["original_timestamp", "latitude", "longitude", "altitude"])

# 遍历bag文件中的消息
for topic, msg, t in bag.read_messages(topics=['/gnss']):  # 替换为你的topic名称
    # 提取原始时间戳（以秒为单位）
    original_timestamp = msg.header.stamp.to_sec()
    # 提取经纬度和高度信息
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude
    # 将数据写入CSV文件
    csv_writer.writerow([original_timestamp, latitude, longitude, altitude])
    # 可选：打印到控制台以便确认
    # print(f"Saved GPS data: timestamp={original_timestamp}, lat={latitude}, lon={longitude}, alt={altitude}")

# 关闭文件
bag.close()
csv_file.close()

print("Data extraction and saving completed.")