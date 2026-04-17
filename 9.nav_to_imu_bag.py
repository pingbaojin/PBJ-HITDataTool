#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 KF_GINS_Navresult.nav 中的姿态角转换为 IMU 四元数并写入 ROS bag。

数据列定义（从 1 开始）:
1: 无意义
2: UTC 时间 (s)
3: 纬度 (deg)
4: 经度 (deg)
5: 椭球高 (m)
6~8: 速度 N/E/D (m/s)
9~11: 姿态角 Roll/Pitch/Yaw (deg), 坐标系为 NED
输出: 转换到 ROS 默认世界系 ENU 后的四元数

个人理解：正东为0度，逆时针为正

把 nav 姿态“生成为新 IMU 话题 bag（弃用，后面转为tf）
""" 

import math
import os

import rosbag
import rospy
from sensor_msgs.msg import Imu
import yaml



def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """
    按 ZYX 顺序（yaw-pitch-roll）将欧拉角转换为四元数。
    输入单位: 度
    输出顺序: x, y, z, w
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def normalize_angle_deg(angle_deg):
    """将角度归一化到 [-180, 180)"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def ned_euler_to_ros_enu_euler(roll_ned, pitch_ned, yaw_ned):
    """
    将 NED 姿态角转换为 ROS 默认 ENU 姿态角（单位: deg）。
    常用对应关系:
      roll_enu  = roll_ned
      pitch_enu = -pitch_ned
      yaw_enu   = 90 - yaw_ned
    """
    roll_enu = roll_ned
    pitch_enu = -pitch_ned
    yaw_enu = normalize_angle_deg(90.0 - yaw_ned)
    return roll_enu, pitch_enu, yaw_enu


def parse_nav_line(line):
    """
    解析一行 nav 数据，返回 (utc_s, roll_deg, pitch_deg, yaw_deg)。
    """
    parts = line.strip().split()
    if len(parts) < 11:
        return None

    try:
        utc_s = float(parts[1])
        roll_deg = float(parts[8])
        pitch_deg = float(parts[9])
        yaw_deg = float(parts[10])
    except ValueError:
        return None

    return utc_s, roll_deg, pitch_deg, yaw_deg


def nav_to_bag(nav_file, bag_file, topic, frame_id):
    if not os.path.exists(nav_file):
        raise FileNotFoundError("找不到输入文件: {}".format(nav_file))

    msg_count = 0

    with rosbag.Bag(bag_file, "w") as bag, open(nav_file, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            # 常见注释/表头行直接跳过
            if line.startswith("#") or line.startswith("%"):
                continue

            parsed = parse_nav_line(line)
            if parsed is None:
                continue

            utc_s, roll_ned, pitch_ned, yaw_ned = parsed
            roll_enu, pitch_enu, yaw_enu = ned_euler_to_ros_enu_euler(
                roll_ned, pitch_ned, yaw_ned
            )
            qx, qy, qz, qw = euler_to_quaternion(roll_enu, pitch_enu, yaw_enu)

            imu = Imu()
            imu.header.stamp = rospy.Time.from_sec(utc_s)
            imu.header.frame_id = frame_id
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw

            # 仅写姿态，其他量未知，按 IMU 约定置为不可用
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0

            bag.write(topic, imu, t=imu.header.stamp)
            msg_count += 1

    return msg_count


config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
with open(config_path, "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)
    
    
    path = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"

    
    nav = path + "KF_GINS_Navresult.nav"
    bag = path + "KF_GINS_Navresult_imu.bag"
    topic = "/imu/data"
    frame_id = "fog"

    count = nav_to_bag(nav, bag, topic, frame_id)
    print("完成: 共写入 {} 条 Imu 到 {}".format(count, bag))