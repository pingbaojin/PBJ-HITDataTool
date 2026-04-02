# !/usr/bin/env python
# docker fastlio2运行
# @pbj
# 2025.04.03
# 输出频率1Hz
import rosbag
import yaml

# 从YAML文件加载数据
with open("./config.yaml", "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

bagpath = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+".bag"
txt_file = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"+"GNSS-RTK"+str(data['bag_name'])+".txt"
""
# 打开bag文件
bag = rosbag.Bag(bagpath)  # 替换为你的bag文件路径
# bag = rosbag.Bag('/home/mars_ugv/docker_ws/data/2025-03-25/2025-03-25-20-39-22.bag')  # 替换为你的bag文件路径

# 打开CSV文件以写入模式
# 创建txt文件
tt = 0
with open(txt_file, 'w') as txtfile:
    # 遍历bag文件中的消息
    for topic, msg, t in bag.read_messages(topics=['/fix']):  # 替换为你的topic名称
        if (tt%100) == 0:
            # 提取原始时间戳（以秒为单位）
            original_timestamp = msg.header.stamp.to_sec()
            # 提取 IMU 数据信息
            lat = msg.latitude
            long = msg.longitude
            height = msg.altitude

            # 将数据写入txt文件
            txtfile.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(original_timestamp, lat, long,height, 0, 0, 0))
        tt = tt + 1
        
        # 可选：打印到控制台以便确认
        # print(f"Saved IMU data: {original_timestamp}\t{wy}\t{wx}\t{-wz}\t{ay}\t{ax}\t{-az}\n")

# 关闭文件
bag.close()

print("Data extraction and saving completed.")