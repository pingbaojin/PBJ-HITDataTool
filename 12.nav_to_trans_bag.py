#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将载体在全局坐标系下的位姿转为 ROS TF bag（话题 /tf，tf2_msgs/TFMessage）。

输入（KF_GINS_Navresult.nav，与 KF-GINS 输出一致）:
  - 全局位置: WGS84 纬度、经度 (deg)、椭球高 (m) —— 列 3~5
  - 全局姿态: 当地导航系 NED 下 Roll/Pitch/Yaw (deg) —— 列 9~11
  - 时间: UTC 时间 (s) —— 列 2
  - 列 6~8 为速度，本脚本不使用

输出:
  - 在「第一帧」经纬高处建立固定局部东北天 (ENU) 系，父坐标系名为 world（原点=首帧位置）；
  - 子坐标系为 base_link（载体）；
  - 平移: 当前全局位置相对首帧在局部 ENU 下的位移 (m)，首帧为 (0,0,0)；
  - 旋转: NED 欧拉角经映射后转为 ROS ENU 下四元数 (ZYX)。

每条消息为 TFMessage，内含一条 TransformStamped，写入 /tf，rosbag play 可更新 TF 树。
"""

from __future__ import print_function

import math
import os

import rosbag
import rospy
import yaml
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# ---------------------------------------------------------------------------
# WGS84 椭球局部平面近似（与 5.plot_navresult 一致）
# ---------------------------------------------------------------------------
WGS84_RA = 6378137.0
WGS84_E1 = 0.00669437999013


def _meridian_prime_vertical_radii(lat_rad):
    """子午圈半径 Rm、卯酉圈半径 Rn。lat_rad: 纬度 [rad]"""
    s = math.sin(lat_rad)
    tmp = 1.0 - WGS84_E1 * (s * s)
    sqrttmp = math.sqrt(tmp)
    rm = WGS84_RA * (1.0 - WGS84_E1) / (sqrttmp * tmp)
    rn = WGS84_RA / sqrttmp
    return rm, rn


def _geodetic_delta_to_ned_m(ref_blh_rad, delta_lat_rad, delta_lon_rad, delta_h_m):
    """
    相对参考点的经纬高增量 -> 局部北、东、地位移 (m)。
    ref_blh_rad: [lat, lon, h] 参考点（弧度, 弧度, 米）
    """
    rm, rn = _meridian_prime_vertical_radii(ref_blh_rad[0])
    north = delta_lat_rad * (rm + ref_blh_rad[2])
    east = delta_lon_rad * (rn + ref_blh_rad[2]) * math.cos(ref_blh_rad[0])
    down = -delta_h_m
    return north, east, down


def _ned_delta_to_ros_enu_xyz(north_m, east_m, down_m):
    """ROS 标准 ENU: x=东, y=北, z=天"""
    return east_m, north_m, -down_m


def _normalize_deg(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


def _ned_euler_deg_to_ros_enu_euler_deg(roll_ned, pitch_ned, yaw_ned):
    """NED 导航姿态 (deg) -> 脚本采用的 ROS ENU 欧拉角 (deg)。"""
    roll_enu = roll_ned
    pitch_enu = -pitch_ned
    yaw_enu = _normalize_deg(90.0 - yaw_ned)
    return roll_enu, pitch_enu, yaw_enu


def _ros_enu_euler_deg_to_quaternion_xyzw(roll_deg, pitch_deg, yaw_deg):
    """
    ENU 下 ZYX（yaw -> pitch -> roll）欧拉角 (deg) -> 四元数 x,y,z,w。
    与 9.nav_to_imu_bag.py 一致。
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


def _parse_nav_line(line):
    """
    解析一行 KF_GINS nav：全局位置 + NED 姿态 + 时间。
    返回 None 表示跳过。
    """
    parts = line.strip().split()
    if len(parts) < 11:
        return None
    try:
        return {
            "utc_s": float(parts[1]),
            "lat_deg": float(parts[2]),
            "lon_deg": float(parts[3]),
            "h_m": float(parts[4]),
            "roll_ned_deg": float(parts[8]),
            "pitch_ned_deg": float(parts[9]),
            "yaw_ned_deg": float(parts[10]),
        }
    except ValueError:
        return None


def _load_global_pose_rows(nav_path):
    """读取 nav 文件中全部有效全局位姿样本。"""
    rows = []
    with open(nav_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("%"):
                continue
            rec = _parse_nav_line(line)
            if rec is not None:
                rows.append(rec)
    return rows


def _global_pose_to_transform_stamped(
    rec,
    ref_blh_rad,
    first_h_m,
):
    """
    单帧：全局 WGS84 + NED 姿态 -> TransformStamped（局部 ENU 相对 ref）。
    """
    lat_rad = math.radians(rec["lat_deg"])
    lon_rad = math.radians(rec["lon_deg"])
    dlat = lat_rad - ref_blh_rad[0]
    dlon = lon_rad - ref_blh_rad[1]
    dh = rec["h_m"] - first_h_m

    north, east, down = _geodetic_delta_to_ned_m(ref_blh_rad, dlat, dlon, dh)
    ex, ny, uz = _ned_delta_to_ros_enu_xyz(north, east, down)

    r_e, p_e, y_e = _ned_euler_deg_to_ros_enu_euler_deg(
        rec["roll_ned_deg"],
        rec["pitch_ned_deg"],
        rec["yaw_ned_deg"],
    )
    qx, qy, qz, qw = _ros_enu_euler_deg_to_quaternion_xyzw(r_e, p_e, y_e)

    ts = TransformStamped()
    ts.header.stamp = rospy.Time.from_sec(rec["utc_s"])
    ts.transform.translation.x = ex
    ts.transform.translation.y = ny
    ts.transform.translation.z = uz
    ts.transform.rotation.x = qx
    ts.transform.rotation.y = qy
    ts.transform.rotation.z = qz
    ts.transform.rotation.w = qw
    return ts


def write_global_pose_nav_to_tf_bag(
    nav_file,
    bag_file,
    tf_topic,
    parent_frame,
    child_frame,
):
    """
    从 KF_GINS_Navresult.nav（全局经纬高 + NED 姿态）写入 TF bag。

    :param nav_file: 输入 .nav 路径
    :param bag_file: 输出 .bag 路径
    :param tf_topic: 一般为 /tf
    :param parent_frame: 局部 ENU 父系名（默认 world）
    :param child_frame: 载体子系名（默认 base_link）
    :return: 写入的消息条数
    """
    if not os.path.exists(nav_file):
        raise FileNotFoundError("找不到输入文件: {}".format(nav_file))

    rows = _load_global_pose_rows(nav_file)
    if not rows:
        raise ValueError("未解析到任何有效 nav 行: {}".format(nav_file))

    ref = rows[0]
    ref_blh_rad = [
        math.radians(ref["lat_deg"]),
        math.radians(ref["lon_deg"]),
        ref["h_m"],
    ]
    h0 = ref["h_m"]

    count = 0
    with rosbag.Bag(bag_file, "w") as bag:
        for rec in rows:
            ts = _global_pose_to_transform_stamped(rec, ref_blh_rad, h0)
            ts.header.frame_id = parent_frame
            ts.child_frame_id = child_frame

            tf_msg = TFMessage()
            tf_msg.transforms = [ts]
            bag.write(tf_topic, tf_msg, t=ts.header.stamp)
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
    nav_path = base + "KF_GINS_Navresult.nav"
    out_bag = base + "KF_GINS_Navresult_trans.bag"
    tf_topic = "/tf"
    # 与 14.read_trans_bag 默认一致；若需改为 map/world 请同步修改合并与绘图脚本
    parent_frame = "world"
    child_frame = "base_link"

    n = write_global_pose_nav_to_tf_bag(
        nav_path,
        out_bag,
        tf_topic,
        parent_frame,
        child_frame,
    )
    print(
        "完成: 共写入 {} 条 TFMessage (全局位姿 -> {} -> {}) 到 {}".format(
            n, parent_frame, child_frame, out_bag
        )
    )
    print("话题: {} (tf2_msgs/TFMessage)".format(tf_topic))
