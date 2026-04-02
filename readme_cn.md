## PBJ-HITDataTool 项目说明（可直接贴到 `readme.md`）

### 1. 项目概述
该项目用于处理与复现一条实验数据的完整链路：从 ROS1 的 `.bag` 中提取 IMU/GNSS（以及“真值/参考”），根据双矢量初始对准算法计算 KF-GINS 所需的初始参数（`initatt`、`initpos`），运行 KF-GINS 得到导航结果后，再把 `KF_GINS_Navresult.nav` 中的姿态转换回 ROS IMU 四元数，写出新的 IMU bag，最后进行轨迹/误差可视化，并提供若干辅助脚本（GNSS 导出 CSV、Google 地图轨迹、扫描 bag 的 tf frame_id）。

本项目目前约定：KF-GINS 使用的 IMU 文件来自 `1.FOG_IMU.py`（即生成 `IMU{bag_name}.txt` 的那套坐标轴/符号约定）。

---

### 2. 运行前准备
1. 确保数据目录结构与命名符合脚本约定（至少需要）：
   - bag 文件：`${fastlio_path}${bagdir}/${bag_name}.bag`
   - 输出目录：`${fastlio_path}${bagdir}/${bag_name}/`（用于保存 `IMU*.txt`、`GNSS-RTK*.txt`、`truth*.txt` 等）

2. 确保 ROS1 环境可用（脚本使用 `rosbag`、`rospy`、`sensor_msgs` 等）。
3. 确保 bag 内 topic 名称匹配：
   - `1.FOG_IMU.py` 读取 `/FOGimuRad`
   - `2.gnss.py` 读取 `/fix`
   - `3.truth.py` 读取 `/fix`
   - `7.ROS的gnss数据直接保存.py` 读取 `/gnss`
4. 建议以 `PBJ-HITDataTool` 目录为工作目录运行脚本：因为脚本普遍用的是相对路径 `./config.yaml`、`./kf-gins.yaml`。

---

### 3. `config.yaml` 需要哪些字段
脚本读取 `./config.yaml`（要求文件在脚本运行目录下），字段如下：

- `fastlio_path`：bag 和 txt 输出的根路径（给 `1~6、9、10、11` 等使用）
- `bagdir`：车/实验类别子目录名（拼接到 `fastlio_path` 下）
- `bag_name`：具体 bag 名称（同时用于 bag 文件名和输出文件名）
- `ubuntu24.04_path`：给 `8.google.py` 使用的路径（用于定位导出的 csv 与生成 html）

示例（来自仓库配置格式）：
```yaml
fastlio_path: /docker_ws/ubuntu20_04/data/HIT/
ubuntu24.04_path: /home/pbj/docker_ws/ubuntu20_04/data/HIT/
bagdir: 2025-04-18-car1
bag_name: 2025-04-18-13-30-35
```

---

### 4. 推荐运行顺序（完整链路）
下面按脚本编号给出推荐流程（从提取开始，到对准、生成 IMU bag、合并、可视化结束）：

1. 提取 IMU 为 KF-GINS 格式（本项目约定使用此脚本）
```bash
python3 1.FOG_IMU.py
```

2. 提取 GNSS（用于初始位置/真值参考）
```bash
python3 2.gnss.py
```

3. 生成 truth 文件（当前版本以 `/fix` 派生，且多字段为 0）
```bash
python3 3.truth.py
```

4. 双矢量初始对准，生成 KF-GINS 配置（写出 `{bag_name}.yaml`）
```bash
python3 4.pbj_align.py
```
输出会用于你把初始对准结果“拷贝到 KF-GINS 工程”里运行（脚本在本目录直接产出新 yaml）。

5. 在 KF-GINS 工程中运行解算，得到 `KF_GINS_Navresult.nav`（然后回到本目录执行后续脚本）

6. 把 `KF_GINS_Navresult.nav` 姿态转换为 ROS IMU 四元数，并写出 imu bag
```bash
python3 9.nav_to_imu_bag.py
```
得到：`KF_GINS_Navresult_imu.bag`

7. 合并原始 bag 与解算得到的新 imu bag（仅保留指定 topics）
```bash
python3 10.merge_bags_no_merge_cmd.py
```
得到：`{bag_name}_q.bag`（输出位置在 `${fastlio_path}${bagdir}/${bag_name}/`）

8. 可视化导航结果/误差（两种脚本二选一或都用）
```bash
python3 5.plot_navresult.py
# 或
python3 6.navplot.py
```

9. 可选辅助：导出 GNSS CSV（供轨迹地图）
```bash
python3 7.ROS的gnss数据直接保存.py
```

10. 可选辅助：用 Google 卫星底图画轨迹（需要 `pandas`、`folium`）
```bash
python3 8.google.py
```

11. 可选辅助：扫描合并后的 bag，列出 bag 里用到的 `header.frame_id` 并给出 static tf 的建议 launch 模板
```bash
python3 11.extract_frames.py
```

---

### 5. 每个输出文件的命名规则（按脚本）
以下命名规则以 `config.yaml` 的 `bag_name` 为核心。

1. `1.FOG_IMU.py` 输出
- 路径：`${fastlio_path}${bagdir}/${bag_name}/IMU${bag_name}.txt`
- 命名：`IMU{bag_name}.txt`
- 说明：脚本会对角速度/加速度进行单位换算（除以 100）以及轴重排/符号翻转，以适配 KF-GINS 的 IMU 坐标系约定。

2. `1.FOG_IMU_ENU.py` 输出（备用，不是本项目当前 KF-GINS 默认使用）
- 路径：`${fastlio_path}${bagdir}/${bag_name}/IMU${bag_name}ENU.txt`
- 命名：`IMU{bag_name}ENU.txt`

3. `2.gnss.py` 输出
- 路径：`${fastlio_path}${bagdir}/${bag_name}/GNSS-RTK${bag_name}.txt`
- 命名：`GNSS-RTK{bag_name}.txt`
- 字段结构：`timestamp lat lon height 0 0 0`（tab 分隔）

4. `3.truth.py` 输出
- 路径：`${fastlio_path}${bagdir}/${bag_name}/truth${bag_name}.txt`
- 命名：`truth{bag_name}.txt`
- 字段结构：当前实现为 `2025 timestamp lat lon height 0 ... 0`（多字段为 0，占位）

5. `4.pbj_align.py` 输出
- 写出一个对准后的 KF-GINS 配置文件：
  - 路径：`./{bag_name}.yaml`
  - 命名：`{bag_name}.yaml`
- 同时会生成/覆盖：
  - `./config02.yaml`（从 `./config.yaml` 派生的版本，包含必要路径信息）

6. `9.nav_to_imu_bag.py` 输出
- 路径：`${fastlio_path}${bagdir}/${bag_name}/KF_GINS_Navresult_imu.bag`
- 命名：`KF_GINS_Navresult_imu.bag`
- 内容：写入 topic `/imu/data`，只填姿态四元数，协方差用 `-1.0` 表示未知/无效。

7. `10.merge_bags_no_merge_cmd.py` 输出
- 路径：`${fastlio_path}${bagdir}/${bag_name}/{bag_name}_q.bag`
- 命名：`{bag_name}_q.bag`
- 合并逻辑：把原始 bag 与 `KF_GINS_Navresult_imu.bag` 按脚本指定的 `TARGET_TOPICS` 合并（只保留这些 topic 的消息）。

8. `7.ROS的gnss数据直接保存.py` 输出
- 路径：`${fastlio_path}${bagdir}/${bag_name}/{bag_name}.csv`
- 命名：`{bag_name}.csv`
- 字段：`original_timestamp, latitude, longitude, altitude`

9. `8.google.py` 输出
- 路径：`${ubuntu24.04_path}${bagdir}/${bag_name}/{bag_name}.html`
- 命名：`{bag_name}.html`
- 内容：Google 卫星底图上的轨迹折线与起点终点 marker。

10. `11.extract_frames.py` 输出
- 不生成文件
- 直接打印 frame_id 列表，并为每个 frame（排除 `nav`、`map`）给出 static transform publisher 的建议节点参数。

---

### 6. 常见问题排查（按现象对照）
1. 找不到 bag 文件 / 生成的 txt 文件失败
- 检查 `${fastlio_path}${bagdir}/${bag_name}.bag` 是否存在
- 检查 `${fastlio_path}${bagdir}/${bag_name}/` 目录是否存在（脚本默认要把 txt 写进这个目录）

2. IMU / GNSS 提取为空或文件内容不对
- 检查 topic 是否存在：
  - IMU：`/FOGimuRad`
  - GNSS：`/fix`
  - 导出 CSV：`/gnss`
- 若 topic 名称不同，需要修改脚本里的 `read_messages(topics=[...])`

3. 初始对准（`4.pbj_align.py`）姿态不合理（roll/pitch/yaw 跳变或明显偏离）
- `4.pbj_align.py` 使用固定的对准窗口累加 `len=12000`，并且有硬编码缩放因子相关 `0.01`（脚本里按 `.../len/0.01` 处理增量尺度）。
- 如果你的 IMU 数据时间步长（dt）不是 0.01s，初始对准的尺度可能不匹配，需要相应调整脚本逻辑或使用与脚本一致的 IMU 数据定义。

4. KF-GINS 运行失败（配置 yaml 路径/字段不匹配）
- 确认 `4.pbj_align.py` 生成的 `{bag_name}.yaml` 是否已拷贝到 KF-GINS 工程可读取位置
- 确认你的 KF-GINS 工程中用到的 `imupath`、`gnsspath` 指向的文件存在（由 `4.pbj_align.py` 写入）

5. `9.nav_to_imu_bag.py` 输出 IMU 四元数异常
- 检查 `KF_GINS_Navresult.nav` 的列格式是否与脚本假设一致（脚本解析 roll/pitch/yaw 在第 9~11 列，使用下标 8/9/10）
- 坐标系转换固定为 NED->ROS ENU 的映射关系（`pitch_enu=-pitch_ned`、`yaw_enu=90-yaw_ned` 并做归一化）。如果你的 `nav` 姿态坐标系不是 NED/对应约定，需要调整转换函数。

6. 合并 bag 后看不到某些 topic
- `10.merge_bags_no_merge_cmd.py` 的 `TARGET_TOPICS` 是固定列表，只会合并这些 topic
- 若你的数据缺少其中某些 topic，最终合并 bag 里也不会有；必要时需要增删 `TARGET_TOPICS`

7. 可视化误差曲线异常（尤其航向角）
- 误差脚本包含“航向角环绕修正”（>180、<-180 时加减 360），如果你的 yaw 定义范围/单位与脚本不一致，也可能导致误差曲线不正常。

---

### 7. 本项目与 KF-GINS 的 IMU 对齐说明（按你的要求）
- 你当前在 KF-GINS 使用的 IMU 版本：`1.FOG_IMU.py` 生成的 `IMU{bag_name}.txt`
- 相应地：
  - `4.pbj_align.py` 读取的就是 `IMU{bag_name}.txt`（不是 ENU 版本）
  - 如果你未来想切换到 `1.FOG_IMU_ENU.py` 生成的 `IMU{bag_name}ENU.txt`，需要同步检查并修改 `4.pbj_align.py` 中 IMU 文件读取与坐标轴约定，否则 `initatt`/`initpos` 可能错误。
