#!/bin/bash

# 设置 PyTorch 和 LibTorch 库路径
export LD_LIBRARY_PATH=/home/zzz/libtorch/lib:$LD_LIBRARY_PATH

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source /home/zzz/ros2_car/ros2/install/setup.bash

echo "环境已设置，库路径："
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"

# 执行传入的参数，如果没有参数则启动导航
if [ $# -eq 0 ]; then
    echo "启动导航系统..."
    ros2 launch car_nav mapping.launch.py
else
    exec "$@"
fi