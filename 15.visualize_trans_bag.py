#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取 KF_GINS_Navresult_trans.bag（/tf，tf2_msgs/TFMessage），
动态显示世界系 (我人类的, ENU) 与载体系 (base_link) 的位姿关系：
  - 3D：轨迹 + 当前时刻载体坐标轴 (x/y/z) 在父系中的方向；
  - 2D：东-北平面轨迹 + 投影的航向示意。

依赖: rosbag, matplotlib, numpy；可选 tf 用于四元数转旋转矩阵。

用法:
  python3 15.visualize_trans_bag.py
  python3 15.visualize_trans_bag.py /path/to/KF_GINS_Navresult_trans.bag
"""

from __future__ import print_function

import argparse
import math
import os
import sys

import numpy as np
import rosbag
import yaml

try:
    import tf.transformations as tft
except ImportError:
    tft = None

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — register 3d projection


def quat_xyzw_to_R(qx, qy, qz, qw):
    """geometry_msgs 四元数 (x,y,z,w) -> 3x3 旋转矩阵 R，使 p_parent = R @ p_child + t。"""
    if tft is not None:
        M = tft.quaternion_matrix([qx, qy, qz, qw])
        return np.array(M[:3, :3], dtype=float)

    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-20:
        return np.eye(3)
    s = 1.0 / math.sqrt(n)
    qx, qy, qz, qw = qx * s, qy * s, qz * s, qw * s
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def load_tf_from_bag(bag_path, parent_frame="world", child_frame="base_link", topic="/tf"):
    """读取 bag 中指定父子系的位姿序列。"""
    if not os.path.exists(bag_path):
        raise FileNotFoundError(bag_path)

    t_list = []
    p_list = []
    q_list = []

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            for ts in msg.transforms:
                if ts.header.frame_id != parent_frame:
                    continue
                if ts.child_frame_id != child_frame:
                    continue
                tr = ts.transform.translation
                q = ts.transform.rotation
                t_list.append(ts.header.stamp.to_sec())
                p_list.append([tr.x, tr.y, tr.z])
                q_list.append([q.x, q.y, q.z, q.w])

    if not t_list:
        raise ValueError(
            "No transform {} -> {} on topic {}".format(
                parent_frame, child_frame, topic
            )
        )

    t0 = t_list[0]
    t_rel = np.array([t - t0 for t in t_list], dtype=float)
    pos = np.array(p_list, dtype=float)
    quat = np.array(q_list, dtype=float)

    return {
        "t_rel": t_rel,
        "pos": pos,
        "quat": quat,
        "parent": parent_frame,
        "child": child_frame,
    }


def subsample(data, max_frames=4000):
    """过长序列降采样以减轻动画负担。"""
    n = len(data["t_rel"])
    if n <= max_frames:
        return data
    idx = np.linspace(0, n - 1, max_frames, dtype=int)
    return {
        "t_rel": data["t_rel"][idx],
        "pos": data["pos"][idx],
        "quat": data["quat"][idx],
        "parent": data["parent"],
        "child": data["child"],
    }


def _set_line3d(line, x, y, z):
    """兼容不同 matplotlib 版本的 3D 折线更新。"""
    if hasattr(line, "set_data_3d"):
        line.set_data_3d(x, y, z)
    else:
        line.set_data(x, y)
        line.set_3d_properties(z)


def run_animation(data, axis_scale=3.0, interval_ms=40):
    """
    data: load_tf_from_bag 返回（可已 subsample）。
    axis_scale: 载体坐标轴箭头长度 (m)。
    """
    pos = data["pos"]
    quat = data["quat"]
    t_rel = data["t_rel"]
    parent = data["parent"]
    child = data["child"]
    n = len(t_rel)

    fig = plt.figure(figsize=(12, 5))
    ax3d = fig.add_subplot(121, projection="3d")
    ax2d = fig.add_subplot(122)

    (line3d,) = ax3d.plot([], [], [], "b-", lw=1.2, label="trajectory")
    (triad_x,) = ax3d.plot([], [], [], "r-", lw=2)
    (triad_y,) = ax3d.plot([], [], [], "g-", lw=2)
    (triad_z,) = ax3d.plot([], [], [], "m-", lw=2)
    (pt3d,) = ax3d.plot([], [], [], "ko", ms=5)

    (line2d,) = ax2d.plot([], [], "b-", lw=1.2)
    (dir2d,) = ax2d.plot([], [], "r-", lw=2)

    ax3d.set_xlabel("East x [m]")
    ax3d.set_ylabel("North y [m]")
    ax3d.set_zlabel("Up z [m]")
    ax3d.set_title("World: {} (ENU)".format(parent))

    margin = axis_scale * 2
    pmin = pos.min(axis=0) - margin
    pmax = pos.max(axis=0) + margin
    ax3d.set_xlim(pmin[0], pmax[0])
    ax3d.set_ylim(pmin[1], pmax[1])
    ax3d.set_zlim(pmin[2], pmax[2])

    ax2d.set_xlabel("East x [m]")
    ax2d.set_ylabel("North y [m]")
    ax2d.set_title("Plan view + body x in EN")
    ax2d.set_aspect("equal")
    ax2d.grid(True)
    xy = pos[:, :2]
    ax2d.set_xlim(xy[:, 0].min() - margin, xy[:, 0].max() + margin)
    ax2d.set_ylim(xy[:, 1].min() - margin, xy[:, 1].max() + margin)

    fig.suptitle(
        "TF: {} -> {}  (child pose in parent frame)".format(parent, child),
        fontsize=11,
    )
    fig.tight_layout()

    time_text = fig.text(0.02, 0.02, "", fontsize=9)

    def update(frame):
        i = int(frame)
        o = pos[i]
        R = quat_xyzw_to_R(quat[i, 0], quat[i, 1], quat[i, 2], quat[i, 3])
        ex = R[:, 0] * axis_scale
        ey = R[:, 1] * axis_scale
        ez = R[:, 2] * axis_scale

        _set_line3d(
            line3d,
            pos[: i + 1, 0],
            pos[: i + 1, 1],
            pos[: i + 1, 2],
        )

        _set_line3d(
            triad_x,
            [o[0], o[0] + ex[0]],
            [o[1], o[1] + ex[1]],
            [o[2], o[2] + ex[2]],
        )
        _set_line3d(
            triad_y,
            [o[0], o[0] + ey[0]],
            [o[1], o[1] + ey[1]],
            [o[2], o[2] + ey[2]],
        )
        _set_line3d(
            triad_z,
            [o[0], o[0] + ez[0]],
            [o[1], o[1] + ez[1]],
            [o[2], o[2] + ez[2]],
        )

        _set_line3d(pt3d, [o[0]], [o[1]], [o[2]])

        line2d.set_data(pos[: i + 1, 0], pos[: i + 1, 1])
        # 2D: body x-axis horizontal projection
        dir2d.set_data([o[0], o[0] + ex[0]], [o[1], o[1] + ex[1]])

        time_text.set_text(
            "t = {:.2f} s  |  frame {}/{}".format(t_rel[i], i + 1, n)
        )
        return line3d, triad_x, triad_y, triad_z, pt3d, line2d, dir2d, time_text

    anim = FuncAnimation(
        fig,
        update,
        frames=n,
        interval=interval_ms,
        blit=False,
        repeat=True,
    )
    plt.show()
    return anim


def main():
    parser = argparse.ArgumentParser(
        description="Animate world (world ENU) vs body (base_link) from trans bag."
    )
    parser.add_argument("bag", nargs="?", default=None, help="Path to .bag file")
    parser.add_argument("--parent", default="world", help="Parent frame_id")
    parser.add_argument("--child", default="base_link", help="child_frame_id")
    parser.add_argument(
        "--scale",
        type=float,
        default=3.0,
        help="Body axis arrow length in meters",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=4000,
        help="Subsample if more poses (animation performance)",
    )
    args = parser.parse_args()

    if args.bag:
        bag_path = args.bag
    else:
        config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "config.yaml"
        )
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        base = (
            str(cfg["fastlio_path"])
            + str(cfg["bagdir"])
            + "/"
            + str(cfg["bag_name"])
            + "/"
        )
        bag_path = base + "KF_GINS_Navresult_trans.bag"

    print("Loading:", bag_path)
    data = load_tf_from_bag(bag_path, args.parent, args.child)
    n_raw = len(data["t_rel"])
    print("Poses:", n_raw)
    data = subsample(data, max_frames=args.max_frames)
    if len(data["t_rel"]) < n_raw:
        print("Subsampled for animation:", len(data["t_rel"]))

    run_animation(data, axis_scale=args.scale)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
