
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v4


# docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 9999 -v4
# docker打开micro-ros环境与小车通讯在9999端口

# ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/zzz/ros2_car/ros2/maps-2/my_map'}}"

# slam建图完后保存地图

# Cartographer保存地图命令:
# mkdir -p /home/zzz/ros2_car/ros2/maps-2
# ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/zzz/ros2_car/ros2/maps-2/my_map.pbstream'}"
# ros2 run nav2_map_server map_saver_cli -f /home/zzz/ros2_car/ros2/maps-2/my_map


# 小车固件一键烧录命令，在platformio下
# python3 ~/.platformio/packages/tool-esptoolpy/esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 460800 write_flash 0x0 /home/zzz/ros2_car/ros2car_backup.bin

# 小车录bag命令:       名字是0925_mission_2
# ros2 bag record -o 0925_mission_2 /cmd_vel /odometry/filtered /scan /imu /tf /tf_static