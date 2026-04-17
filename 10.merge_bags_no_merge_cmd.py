"""
把这个新 IMU bag 和原始传感器 bag 合并成一个可直接用的总 bag（弃用）
"""


import rosbag
import os
import yaml

def merge_specific_topics(bag_paths, output_path, target_topics):
    """
    合并多个 bag 文件的指定话题（直接合并，无需提前过滤）
    :param bag_paths: 待合并的 bag 文件路径列表（如 ["bag1.bag", "bag2.bag"]）
    :param output_path: 合并后的输出路径
    :param target_topics: 需要保留的话题列表
    """
    # 检查输入文件是否存在
    for bag_path in bag_paths:
        if not os.path.exists(bag_path):
            print(f"错误：文件 {bag_path} 不存在！")
            return

    # 打开输出 bag（写入模式）
    with rosbag.Bag(output_path, 'w') as out_bag:
        total_msgs = 0  # 统计合并的消息总数
        for idx, bag_path in enumerate(bag_paths):
            print(f"正在处理第 {idx+1} 个 bag：{bag_path}...")
            # 读取当前 bag，只筛选目标话题
            with rosbag.Bag(bag_path, 'r') as in_bag:
                # read_messages(topics=...) 直接过滤话题，无需额外判断
                msg_count = 0
                for topic, msg, t in in_bag.read_messages(topics=target_topics):
                    out_bag.write(topic, msg, t)
                    msg_count += 1
                total_msgs += msg_count
                print(f"  - 该 bag 中提取到 {msg_count} 条目标话题消息")

    print(f"\n合并完成！")
    print(f"  - 输出文件：{output_path}")
    print(f"  - 合并的总消息数：{total_msgs}")

if __name__ == '__main__':
    with open("./config.yaml", "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)    
    path1 = str(data['fastlio_path'])+str(data['bagdir'])+"/"
    path2 = str(data['fastlio_path'])+str(data['bagdir'])+"/"+str(data['bag_name'])+"/"
    
    bag1 = path1 + str(data['bag_name']) + ".bag"
    bag2 = path2 + "KF_GINS_Navresult_imu.bag"
    
    # ===================== 配置区（修改为你的实际信息）=====================
    INPUT_BAGS = [bag1, bag2]  # 待合并的两个 bag 文件路径
    OUTPUT_BAG = path2 + str(data['bag_name']) + "_q.bag"
    TARGET_TOPICS = ["/FOGimuRad", "/MEMSimuRad","/fix", "/vel","/lslidar_point_cloud", "/imu/data"]    # 需要合并的特定话题
    # =====================================================================

    # 执行合并
    merge_specific_topics(INPUT_BAGS, OUTPUT_BAG, TARGET_TOPICS)
