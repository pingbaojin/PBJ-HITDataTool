#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import sys
import yaml

def extract_bag_frames(bag_path):
    try:
        bag = rosbag.Bag(bag_path)
    except Exception as e:
        print("读取 Bag 失败: {}".format(e))
        return

    # 使用集合去重
    frames_found = set()
    topic_frame_map = {}

    print("正在扫描 Bag: {} ...\n".format(bag_path))

    # 遍历 Bag 中所有话题的元数据
    type_and_topics = bag.get_type_and_topic_info().topics
    
    for topic, info in type_and_topics.items():
        # 只扫描有数据的话题，取第一条消息
        for _, msg, _ in bag.read_messages(topics=topic):
            # 检查消息是否有 header 属性且 header 有 frame_id
            if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                frame_id = msg.header.frame_id
                if frame_id: # 过滤空字符串
                    frames_found.add(frame_id)
                    topic_frame_map[topic] = frame_id
            # 拿到第一条后就跳出当前话题，提高速度
            break

    bag.close()

    # 打印结果
    print("=== 扫描结果 ===")
    print("{:<30} | {:<20}".format("话题 (Topic)", "坐标系 (Frame ID)"))
    print("-" * 55)
    for topic in sorted(topic_frame_map.keys()):
        print("{:<30} | {:<20}".format(topic, topic_frame_map[topic]))

    print("\n=== 建议的 Launch 模板 (静态转换) ===")
    for frame in sorted(frames_found):
        if frame != "nav" and frame != "map": # 过滤掉可能的根节点
            print('<node pkg="tf2_ros" type="static_transform_publisher" name="base_to_{0}" args="0 0 0 0 0 0 base_link {0}" />'.format(frame))

if __name__ == "__main__":
    with open("./config.yaml", "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)    
    path1 = str(data['fastlio_path'])+str(data['bagdir'])+"/"
    path2 = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"
    OUTPUT_BAG = path2 + str(data['bag_name']) + "_q.bag"
    
    extract_bag_frames(OUTPUT_BAG)