# src/process_bags.py (最终版)
"""
ROS2 Bag 数据处理脚本

功能说明：
该脚本用于处理 ROS2 bag 文件中的传感器数据，为端到端自主驾驶训练准备数据集。
主要处理激光雷达、IMU、里程计和控制命令数据，生成用于训练的同步数据样本。

处理流程：
1. 从多个 bag 文件中读取传感器数据
2. 同步不同传感器的时间戳
3. 计算未来目标位置（基于当前姿态和轨迹）
4. 保存处理后的数据为 numpy 数组格式

输出文件：
- scans.npy: 激光雷达扫描数据
- imus.npy: IMU 数据（角速度z轴、线加速度x轴）
- goals.npy: 局部坐标系中的目标位置
- actions.npy: 控制动作（线速度、角速度）
"""

import numpy as np
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from scipy.spatial.transform import Rotation as R
from rclpy.serialization import deserialize_message

# 导入消息类型
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist

# =============================================================================
# 1. 配置参数
# =============================================================================
"""
配置参数模块：定义数据路径、话题名称和处理参数
"""
BAG_PARENT_DIR = Path('../raw_bags')          # 原始 bag 文件目录
PROCESSED_DATA_DIR = Path('../processed_data') # 处理后数据保存目录
TOPIC_ODOM = '/odometry/filtered'             # 里程计话题名称
TOPIC_SCAN = '/scan'                          # 激光雷达话题名称
TOPIC_IMU = '/imu'                            # IMU 话题名称
TOPIC_CMD = '/cmd_vel'                        # 控制命令话题名称
LOOKAHEAD_TIME_SEC = 1.0                      # 预测时间窗口（秒）

# =============================================================================
# 2. 主处理函数
# =============================================================================
"""
主处理函数：处理所有 bag 文件并生成训练数据集

功能：
- 遍历所有 bag 文件目录
- 读取和解析传感器消息
- 同步多传感器数据
- 计算目标位置和控制动作
- 保存处理后的数据
"""
def process_all_bags():
    PROCESSED_DATA_DIR.mkdir(exist_ok=True)  # 创建输出目录
    all_scans, all_imus, all_goals, all_actions = [], [], [], []  # 初始化数据列表
    bag_dirs = sorted([d for d in BAG_PARENT_DIR.iterdir() if d.is_dir()])  # 获取所有 bag 目录

    for i, bag_dir in enumerate(bag_dirs):
        print(f"--- Processing Bag {i+1}/{len(bag_dirs)}: {bag_dir.name} ---")

        # --- 修正部分：正确使用 SequentialReader ---
        storage_options = StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

        reader = SequentialReader()
        try:
            reader.open(storage_options, converter_options)

            print("Pass 1: Reading all messages...")
            trajectory, imu_messages, cmd_vel_messages, scan_messages = [], [], [], []  # 初始化消息列表

            # 读取所有消息并分类存储
            while reader.has_next():
                topic_name, msg_bytes, timestamp_ns = reader.read_next()

                if topic_name == TOPIC_ODOM:
                    msg = deserialize_message(msg_bytes, Odometry)
                    pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
                    trajectory.append({'ts': timestamp_ns, 'pos': np.array([pos.x, pos.y, pos.z]), 'ori': np.array([ori.x, ori.y, ori.z, ori.w])})
                elif topic_name == TOPIC_IMU:
                    imu_messages.append({'ts': timestamp_ns, 'msg': deserialize_message(msg_bytes, Imu)})
                elif topic_name == TOPIC_CMD:
                    cmd_vel_messages.append({'ts': timestamp_ns, 'msg': deserialize_message(msg_bytes, Twist)})
                elif topic_name == TOPIC_SCAN:
                    scan_messages.append({'ts': timestamp_ns, 'msg': deserialize_message(msg_bytes, LaserScan)})

            print(f"Trajectory extracted with {len(trajectory)} points.")
            if not trajectory:
                print(f"WARNING: No trajectory data found in {bag_dir.name}, skipping.")
                continue

            print("Pass 2: Synchronizing data and calculating goals...")
            # 提取时间戳数组用于同步
            traj_ts = np.array([p['ts'] for p in trajectory])
            imu_ts = np.array([p['ts'] for p in imu_messages])
            cmd_ts = np.array([p['ts'] for p in cmd_vel_messages])

            # 为每个激光雷达扫描计算对应的目标和动作
            for scan_data in scan_messages:
                current_ts = scan_data['ts']

                # 找到最接近当前时间戳的姿态
                current_pose_idx = np.argmin(np.abs(traj_ts - current_ts))
                current_pose = trajectory[current_pose_idx]

                # 计算未来时间戳（1.5秒后）
                future_ts = current_ts + int(LOOKAHEAD_TIME_SEC * 1e9)
                future_pose_idx = np.argmin(np.abs(traj_ts - future_ts))

                # 如果未来姿态时间戳差距太大，跳过此样本
                if abs(traj_ts[future_pose_idx] - future_ts) > 0.5e9: continue
                future_pose = trajectory[future_pose_idx]

                # 计算全局坐标系中的位置差
                delta_pos_global = future_pose['pos'] - current_pose['pos']
                # 转换为局部坐标系
                current_rotation = R.from_quat(current_pose['ori'])
                goal_local = current_rotation.inv().apply(delta_pos_global)

                # 找到最接近的 IMU 和控制命令数据
                imu_idx = np.argmin(np.abs(imu_ts - current_ts))
                cmd_idx = np.argmin(np.abs(cmd_ts - current_ts))

                # 收集数据样本
                all_scans.append(scan_data['msg'].ranges)  # 激光雷达距离数据
                imu_msg = imu_messages[imu_idx]['msg']
                all_imus.append([imu_msg.angular_velocity.z, imu_msg.linear_acceleration.x])  # IMU 数据
                all_goals.append(goal_local[:2])  # 局部目标位置 (x, y)
                cmd_msg = cmd_vel_messages[cmd_idx]['msg']
                all_actions.append([cmd_msg.linear.x, cmd_msg.angular.z])  # 控制动作

        except Exception as e:
            print(f"Error processing bag {bag_dir.name}: {e}")
            continue
        finally:
            # 确保 reader 被正确关闭
            if 'reader' in locals():
                del reader

    print("\n--- All bags processed ---")

    # 保存处理后的数据
    if all_scans:
        print(f"Saving {len(all_scans)} samples...")
        np.save(PROCESSED_DATA_DIR / 'scans.npy', np.array(all_scans, dtype=np.float32))
        np.save(PROCESSED_DATA_DIR / 'imus.npy', np.array(all_imus, dtype=np.float32))
        np.save(PROCESSED_DATA_DIR / 'goals.npy', np.array(all_goals, dtype=np.float32))
        np.save(PROCESSED_DATA_DIR / 'actions.npy', np.array(all_actions, dtype=np.float32))
        print("Data saved successfully!")
    else:
        print("No valid samples were generated.")

# =============================================================================
# 3. 主程序入口
# =============================================================================
"""
主程序入口：当脚本直接运行时调用主处理函数
"""
if __name__ == '__main__':
    process_all_bags()