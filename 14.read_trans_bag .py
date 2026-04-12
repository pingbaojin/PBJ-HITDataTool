#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 KF_GINS_Navresult_trans.bag（话题 /tf，tf2_msgs/TFMessage），
解析 gps -> base_link 的 Transform，绘制位置、姿态随时间变化曲线。

与 12.nav_to_trans_bag.py 配套：默认匹配 frame_id=gps, child_frame_id=base_link。
"""

from __future__ import print_function

import math
import os
import sys

import matplotlib.pyplot as plt
from matplotlib import font_manager
import rosbag
import yaml

try:
    import tf.transformations as tft
except ImportError:
    tft = None


def _setup_matplotlib_chinese():
    """
    避免图中中文标题/坐标轴显示为方框：选用系统已安装的支持中文的字体。
    并关闭 Unicode 负号用 ASCII 减号替代，防止中文环境下负号异常。
    """
    keywords = (
        "Noto Sans CJK",
        "Noto Serif CJK",
        "Source Han Sans SC",
        "Source Han Sans CN",
        "WenQuanYi",
        "WQY",
        "SimHei",
        "Microsoft YaHei",
        "PingFang SC",
        "STHeiti",
        "AR PL UMing",
    )
    chosen = None
    for f in font_manager.fontManager.ttflist:
        name = f.name
        for kw in keywords:
            if kw.lower() in name.lower():
                chosen = name
                break
        if chosen:
            break
    if chosen:
        plt.rcParams["font.sans-serif"] = [chosen] + list(
            plt.rcParams.get("font.sans-serif", [])
        )
        plt.rcParams["font.family"] = "sans-serif"
    else:
        import warnings

        warnings.warn(
            "未检测到常见中文字体，图中中文可能仍显示为方框。"
            "Debian/Ubuntu 可执行: sudo apt install fonts-noto-cjk",
            UserWarning,
        )
    plt.rcParams["axes.unicode_minus"] = False


_setup_matplotlib_chinese()


def quat_xyzw_to_euler_rpy(qx, qy, qz, qw):
    """
    四元数 (x,y,z,w) -> 欧拉角 roll, pitch, yaw（弧度），ZYX 顺序，与 tf.transformations 常见用法一致。
    若无 tf 模块则用手算。
    """
    if tft is not None:
        roll, pitch, yaw = tft.euler_from_quaternion([qx, qy, qz, qw], axes="sxyz")
        return roll, pitch, yaw

    # fallback: ZYX from quaternion
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def unwrap_angle_rad(a):
    """简单展开避免 yaw 曲线在 ±pi 处断裂（用于绘图）。"""
    out = [a[0]]
    for i in range(1, len(a)):
        d = a[i] - out[-1]
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        out.append(out[-1] + d)
    return out


def read_trans_bag(
    bag_path,
    parent_frame="gps",
    child_frame="base_link",
    topic="/tf",
):
    if not os.path.exists(bag_path):
        raise FileNotFoundError("找不到 bag: {}".format(bag_path))

    t_list = []
    x_list = []
    y_list = []
    z_list = []
    roll_deg = []
    pitch_deg = []
    yaw_deg = []

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            for ts in msg.transforms:
                if ts.header.frame_id != parent_frame:
                    continue
                if ts.child_frame_id != child_frame:
                    continue

                sec = ts.header.stamp.to_sec()
                tr = ts.transform.translation
                q = ts.transform.rotation
                qx, qy, qz, qw = q.x, q.y, q.z, q.w

                r, p, y = quat_xyzw_to_euler_rpy(qx, qy, qz, qw)

                t_list.append(sec)
                x_list.append(tr.x)
                y_list.append(tr.y)
                z_list.append(tr.z)
                roll_deg.append(math.degrees(r))
                pitch_deg.append(math.degrees(p))
                yaw_deg.append(math.degrees(y))

    if not t_list:
        raise ValueError(
            "未读到 {} -> {} 的变换（话题 {}）。请检查 bag 是否与 12 脚本生成一致。".format(
                parent_frame, child_frame, topic
            )
        )

    t0 = t_list[0]
    t_rel = [ti - t0 for ti in t_list]

    yaw_unwrapped = unwrap_angle_rad([math.radians(y) for y in yaw_deg])
    yaw_deg_plot = [math.degrees(w) for w in yaw_unwrapped]

    return {
        "t_abs": t_list,
        "t_rel": t_rel,
        "x": x_list,
        "y": y_list,
        "z": z_list,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg_plot,
    }


def plot_tf_series(data, save_prefix=None):
    t = data["t_rel"]

    fig1, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    ax[0].plot(t, data["x"], "b-", lw=1.0)
    ax[0].set_ylabel("x East [m]")
    ax[0].grid(True)
    ax[1].plot(t, data["y"], "g-", lw=1.0)
    ax[1].set_ylabel("y North [m]")
    ax[1].grid(True)
    ax[2].plot(t, data["z"], "r-", lw=1.0)
    ax[2].set_ylabel("z Up [m]")
    ax[2].set_xlabel("Time [s] (relative to first sample)")
    ax[2].grid(True)
    fig1.suptitle("Position (ENU, parent frame: gps)")
    fig1.tight_layout()

    fig2, ax2 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    ax2[0].plot(t, data["roll_deg"], lw=1.0)
    ax2[0].set_ylabel("Roll [deg]")
    ax2[0].grid(True)
    ax2[1].plot(t, data["pitch_deg"], lw=1.0)
    ax2[1].set_ylabel("Pitch [deg]")
    ax2[1].grid(True)
    ax2[2].plot(t, data["yaw_deg"], lw=1.0)
    ax2[2].set_ylabel("Yaw [deg] (unwrapped)")
    ax2[2].set_xlabel("Time [s] (relative to first sample)")
    ax2[2].grid(True)
    fig2.suptitle("Attitude (Euler ZYX from quaternion, deg)")
    fig2.tight_layout()

    fig3, ax3 = plt.subplots(1, 1, figsize=(8, 8))
    ax3.plot(data["x"], data["y"], "b-", lw=1.0)
    ax3.plot(data["x"][0], data["y"][0], "go", ms=8, label="Start")
    ax3.plot(data["x"][-1], data["y"][-1], "rs", ms=8, label="End")
    ax3.set_xlabel("East x [m]")
    ax3.set_ylabel("North y [m]")
    ax3.set_title("Horizontal trajectory (ENU)")
    ax3.axis("equal")
    ax3.grid(True)
    ax3.legend()
    fig3.tight_layout()

    if save_prefix:
        fig1.savefig(save_prefix + "_position.png", dpi=150)
        fig2.savefig(save_prefix + "_attitude.png", dpi=150)
        fig3.savefig(save_prefix + "_plan.png", dpi=150)
        print("已保存: {}_position.png, {}_attitude.png, {}_plan.png".format(
            save_prefix, save_prefix, save_prefix
        ))

    plt.show()


if __name__ == "__main__":
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    path = (
        str(cfg["fastlio_path"])
        + str(cfg["bagdir"])
        + "/"
        + str(cfg["bag_name"])
        + "/"
    )
    bag_file = path + "KF_GINS_Navresult_trans.bag"
    save_prefix = path + "figures" + "/" + "KF_GINS_Navresult_trans_plot"

    parent = "world"
    child = "base_link"
    if len(sys.argv) >= 2:
        bag_file = sys.argv[1]
    if len(sys.argv) >= 4:
        parent = sys.argv[2]
        child = sys.argv[3]

    print("读取: {}".format(bag_file))
    series = read_trans_bag(bag_file, parent_frame=parent, child_frame=child)
    print("共 {} 条 TF 记录".format(len(series["t_abs"])))

    plot_tf_series(series, save_prefix=save_prefix)
