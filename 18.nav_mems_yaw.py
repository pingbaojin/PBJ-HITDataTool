#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 IMU2025-04-18-13-30-35.txt 与 KF_GINS_Navresult.nav，
输出包含 /mems_radyaw 话题（sensor_msgs/Imu）的 bag。

消息内容：
  - angular_velocity / linear_acceleration：来自 IMU txt
  - orientation（四元数）：由 nav 的 Roll/Pitch/Yaw 生成
"""

from __future__ import print_function

import bisect
import math
import os

import rosbag
import rospy
from sensor_msgs.msg import Imu
import yaml


def _normalize_deg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def _ned_euler_deg_to_ros_enu_euler_deg(roll_ned, pitch_ned, yaw_ned):
    """NED 欧拉角 -> ROS ENU 欧拉角（deg）。"""
    roll_enu = roll_ned
    pitch_enu = -pitch_ned
    yaw_enu = _normalize_deg(90.0 - yaw_ned)
    return roll_enu, pitch_enu, yaw_enu


def _euler_deg_to_quaternion_xyzw(roll_deg, pitch_deg, yaw_deg):
    """按 ZYX（yaw->pitch->roll）将欧拉角转换为四元数。"""
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


def _parse_imu_line(line):
    """
    解析 IMU txt 单行：
    timestamp wx wy wz ax ay az
    """
    parts = line.strip().split()
    if len(parts) < 7:
        return None
    try:
        t = float(parts[0])
        wx = float(parts[1])
        wy = float(parts[2])
        wz = float(parts[3])
        ax = float(parts[4])
        ay = float(parts[5])
        az = float(parts[6])
    except ValueError:
        return None
    return {"t": t, "wx": wx, "wy": wy, "wz": wz, "ax": ax, "ay": ay, "az": az}


def _parse_nav_line(line):
    """
    解析 nav 单行（列从 1 开始）：
    2: UTC(s), 9~11: roll/pitch/yaw(deg, NED)
    """
    parts = line.strip().split()
    if len(parts) < 11:
        return None
    try:
        return {
            "t": float(parts[1]),
            "roll_ned_deg": float(parts[8]),
            "pitch_ned_deg": float(parts[9]),
            "yaw_ned_deg": float(parts[10]),
        }
    except ValueError:
        return None


def _load_rows(path, parser):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("%"):
                continue
            rec = parser(line)
            if rec is not None:
                rows.append(rec)
    return rows


def _nearest_nav(nav_rows, nav_times, t_imu):
    """按时间最近邻匹配 nav 样本。"""
    idx = bisect.bisect_left(nav_times, t_imu)
    if idx <= 0:
        return nav_rows[0]
    if idx >= len(nav_rows):
        return nav_rows[-1]
    prev_rec = nav_rows[idx - 1]
    next_rec = nav_rows[idx]
    if abs(prev_rec["t"] - t_imu) <= abs(next_rec["t"] - t_imu):
        return prev_rec
    return next_rec


def write_mems_radyaw_bag(imu_txt, nav_file, bag_file, topic, frame_id):
    if not os.path.exists(imu_txt):
        raise FileNotFoundError("找不到 IMU 文件: {}".format(imu_txt))
    if not os.path.exists(nav_file):
        raise FileNotFoundError("找不到 nav 文件: {}".format(nav_file))

    imu_rows = _load_rows(imu_txt, _parse_imu_line)
    nav_rows = _load_rows(nav_file, _parse_nav_line)
    if not imu_rows:
        raise ValueError("IMU 文件无有效数据: {}".format(imu_txt))
    if not nav_rows:
        raise ValueError("nav 文件无有效数据: {}".format(nav_file))

    nav_rows.sort(key=lambda r: r["t"])
    nav_times = [r["t"] for r in nav_rows]

    count = 0
    with rosbag.Bag(bag_file, "w") as bag:
        for rec_imu in imu_rows:
            rec_nav = _nearest_nav(nav_rows, nav_times, rec_imu["t"])
            roll_enu, pitch_enu, yaw_enu = _ned_euler_deg_to_ros_enu_euler_deg(
                rec_nav["roll_ned_deg"], rec_nav["pitch_ned_deg"], rec_nav["yaw_ned_deg"]
            )
            qx, qy, qz, qw = _euler_deg_to_quaternion_xyzw(roll_enu, pitch_enu, yaw_enu)

            msg = Imu()
            msg.header.stamp = rospy.Time.from_sec(rec_imu["t"])
            msg.header.frame_id = frame_id
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            msg.angular_velocity.x = rec_imu["wx"]*100.0
            msg.angular_velocity.y = rec_imu["wy"]*100.0
            msg.angular_velocity.z = rec_imu["wz"]*100.0
            msg.linear_acceleration.x = rec_imu["ax"]*100.0
            msg.linear_acceleration.y = rec_imu["ay"]*100.0
            msg.linear_acceleration.z = rec_imu["az"]*100.0

            bag.write(topic, msg, t=msg.header.stamp)
            count += 1
    return count


if __name__ == "__main__":
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    base = (
        str(cfg["fastlio_path"])
        + str(cfg["bagdir"])
        + "/"
        + str(cfg["bag_name"])
        + "/"
    )
    imu_txt_path = base + "IMU2025-04-18-13-30-35ENU.txt"
    nav_path = base + "KF_GINS_Navresult.nav"
    out_bag = base + "IMU2025-04-18-13-30-35_mems_radyaw.bag"
    topic_name = "/mems_radyaw"
    frame = "base_link"

    n = write_mems_radyaw_bag(
        imu_txt=imu_txt_path,
        nav_file=nav_path,
        bag_file=out_bag,
        topic=topic_name,
        frame_id=frame,
    )
    print("完成: 共写入 {} 条 Imu 到 {}".format(n, out_bag))
    print("话题: {} (sensor_msgs/Imu)".format(topic_name))
