#!/bin/bash

# 容器名称
CONTAINER_NAME="fastlio2"

xhost +
# 检查容器是否已经存在
if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo "容器 ${CONTAINER_NAME} 已经在运行。"
else
    if [ "$(docker ps -aq -f status=exited -f name=^${CONTAINER_NAME}$)" ]; then
        echo "容器 ${CONTAINER_NAME} 已存在但已停止，正在重新启动..."
        docker start ${CONTAINER_NAME}
        # 等待容器启动完成
        # echo "等待容器启动完成...
        # sleep 5
    fi
fi

# 定义路径和 Bag 名称
DATA_PATH="/home/pbj/docker_ws/data/2025-03-27-car1"
# DATA_PATH="/media/pbj/pbj/1.data_collect/2025-03-27-car2"
# BAG_NAME="2025-03-27-14-42-13"
# BAG_NAME="2025-03-27-15-05-25"
BAG_NAME="2025-03-27-16-11-22"
# BAG_NAME="2025-03-27-16-11-29"
DOCKER_CODE_PATH="/home/mars_ugv/docker_ws/3.Ros2txt/本地数据/"
MY_CODE_PATH="home/pbj/docker_ws/3.Ros2txt/本地数据"

# 创建一个新的 tmux 会话，命名为 my_session
tmux new-session -d -s fastlio2

# 在第一个窗口中执行命令
tmux send-keys -t fastlio2:0 "cd ${DATA_PATH}" C-m
tmux send-keys -t fastlio2:0 "mkdir ${BAG_NAME}" C-m
echo "创建文件夹成功"


# 定义操作 A 的函数
operation_a() {
    echo "执行 1.提取ros的FOGIMU数据为IMU.txt 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:1 -n "1.FOG_IMU"
    tmux send-keys -t fastlio2:1 "docker exec -it fastlio2 bash" C-m
    tmux send-keys -t fastlio2:1 "cd ${DOCKER_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:1 "/bin/python3 ./1.FOG_IMU.py" C-m
    echo "1.FOG_IMU.py执行完成"
    echo " "
}

# 定义操作 B 的函数
operation_b() {
    echo "执行 2.提取ros的gnss数据为1Hz的GNSS-RTK.txt 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:2 -n "2.GNSS"
    tmux send-keys -t fastlio2:2 "docker exec -it fastlio2 bash" C-m
    tmux send-keys -t fastlio2:2 "cd ${DOCKER_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:2 "/bin/python3 ./2.gnss.py" C-m
    echo "2.gnss.py执行完成"
    echo " "
}

# 定义操作 C 的函数
operation_c() {
    echo "执行 3.提取ros的gnss数据为100Hz的truth.txt 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:3 -n "3.truth"
    tmux send-keys -t fastlio2:3 "docker exec -it fastlio2 bash" C-m
    tmux send-keys -t fastlio2:3 "cd ${DOCKER_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:3 "/bin/python3 ./3.truth.py" C-m
    echo "3.truth.py执行完成"
    echo " "
}

# 定义操作 D 的函数
operation_d() {
    echo "执行 4.初始对准，注意将对准结果写到组合导航KF-GINS 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:4 -n "4.align"
    tmux send-keys -t fastlio2:4 "cd ${MY_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:4 "/home/pbj/anaconda3/envs/pbj/bin/python ./4.pbj_align.py" C-m
    echo "4.pbj_align.py执行完成"
    echo " "
}
# 定义操作 E 的函数
operation_e() {
    echo "执行 5.绘制导航结果 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:5 -n "5.nav"
    tmux send-keys -t fastlio2:5 "cd ${MY_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:5 "/home/pbj/anaconda3/envs/pbj/bin/python ./5.plot_navresult.py" C-m
    echo "5.plot_navresult.py执行完成"
    echo " "
}

# 定义操作 F 的函数
operation_f() {
    echo "执行 6.绘制误差结果 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:6 -n "6.err"
    tmux send-keys -t fastlio2:6 "cd ${MY_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:6 "/home/pbj/anaconda3/envs/pbj/bin/python ./6.navplot.py" C-m
    echo "6.navplot.py执行完成"
    echo " "
}
# 定义操作 G 的函数
operation_g() {
    echo "执行 7.提取ros的位置数据为csv文件 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:7 -n "7.csv"
    tmux send-keys -t fastlio2:7 "docker exec -it fastlio2 bash" C-m
    tmux send-keys -t fastlio2:7 "cd ${DOCKER_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:7 "/bin/python3 ./7.ROS的gnss数据直接保存.py" C-m
    echo "7.ROS的gnss数据直接保存.py执行完成"
    echo " "
}

# 定义操作 H 的函数
operation_h() {
    echo "执行 8.绘制google地图文件 代码"
    # 在这里添加操作 A 的具体代码
    tmux new-window -t fastlio2:6 -n "8.google"
    tmux send-keys -t fastlio2:6 "cd ${MY_CODE_PATH}" C-m
    tmux send-keys -t fastlio2:6 "/home/pbj/anaconda3/envs/pbj/bin/python ./8.google.py" C-m
    echo "8.google.py执行完成"
    echo " "
}


# 使用 while 循环不断获取用户输入
while true; do
    # 提示用户输入
    echo "请输入数字1-8（输入 exit 退出）："
    echo "1.提取ros的FOGIMU数据为IMU.txt"
    echo "2.提取ros的gnss数据为1Hz的GNSS-RTK.txt"
    echo "3.提取ros的gnss数据为100Hz的truth.txt"
    echo "4.初始对准，注意将对准结果写到组合导航KF-GINS"
    echo "5.绘制导航结果"
    echo "6.绘制误差结果"
    echo "7.提取ros的位置数据为csv文件"
    echo "8.绘制google地图文件"
    read user_input
    # 将输入转换为小写，忽略大小写
    user_input=$(echo "$user_input" | tr '[:upper:]' '[:lower:]')

    if [ "$user_input" = "1" ]; then
        operation_a
    elif [ "$user_input" = "2" ]; then
        operation_b
    elif [ "$user_input" = "3" ]; then
        operation_c
    elif [ "$user_input" = "4" ]; then
        operation_d
    elif [ "$user_input" = "5" ]; then
        operation_e
    elif [ "$user_input" = "6" ]; then
        operation_f
    elif [ "$user_input" = "7" ]; then
        operation_g
    elif [ "$user_input" = "8" ]; then
        operation_h
    elif [ "$user_input" = "exit" ]; then
        tmux kill-session -t fastlio2
        echo "kill-session：fastlio2，退出程序。"
        break  # 退出循环
    else
        echo "无效输入，请输入数字1-8。"
    fi
done