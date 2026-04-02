# 1.bag中的IMU数据提取为txt文件
前提： 有按照要求命名的文件夹
```python3
提取为KF-GINS格式的数据
python3 1.FOG_IMU.py
```
# 2.按照此逻辑依次提取gnss，真值（格式不同），和初始对准结果

# 3.将初始对准结果拷贝到kf-gins项目中

# 4.解算完直接将nav转为imu的bag

# 5.合并提取新数据bag
```python3
# 合并两个过滤后的 bag，输出为 merged_bag.bag
rosbag merge /docker_ws/ubuntu20_04/data/HIT/2025-04-18-car1/2025-04-18-13-30-35.bag /docker_ws/ubuntu20_04/data/HIT/2025-04-18-car1/2025-04-18-13-30-35/KF_GINS_Navresult_imu.bag -o /docker_ws/ubuntu20_04/data/HIT/2025-04-18-car1/2025-04-18-13-30-35/2025-04-18-13-30-35_q.bag
```