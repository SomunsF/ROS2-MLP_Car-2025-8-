# src/inference_node.py

import rclpy
from rclpy.node import Node
import numpy as np
import torch
from scipy.spatial.transform import Rotation as R

# 导入我们需要的消息类型和模型定义
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist

# !!! 注意：你需要把之前 train.py 里的 DrivingModel 类定义复制到这里
# 或者是把它放到一个单独的文件里，然后在这里 import
class DrivingModel(torch.nn.Module):
    def __init__(self, input_size, output_size):
        super(DrivingModel, self).__init__()
        self.network = torch.nn.Sequential(
            torch.nn.Linear(input_size, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, output_size)
        )
    def forward(self, x):
        return self.network(x)

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')

        # --- 1. 参数定义 ---
        self.declare_parameter('model_path', '../model_1.5s.pth') # 模型路径
        self.declare_parameter('input_size', 364) # 模型的输入维度
        self.declare_parameter('output_size', 2)  # 模型的输出维度
        self.declare_parameter('lookahead_time_sec', 1.5) # 与训练时匹配的前瞻时间

        # 获取参数
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        input_size = self.get_parameter('input_size').get_parameter_value().integer_value
        output_size = self.get_parameter('output_size').get_parameter_value().integer_value
        self.lookahead_time = self.get_parameter('lookahead_time_sec').get_parameter_value().double_value

        # --- 2. 加载模型 ---
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = DrivingModel(input_size, output_size)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval() # 切换到评估模式
        self.get_logger().info(f"Model loaded from {model_path} on device {self.device}")

        # --- 3. 初始化变量 ---
        self.current_odom = None
        self.current_scan = None
        self.current_imu = None
        self.global_plan = None

        # --- 4. ROS 2 通信 ---
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # 订阅Nav2输出的全局路径
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建一个高频定时器作为主控制循环
        self.control_timer = self.create_timer(0.1, self.control_loop) # 10 Hz

    def odom_callback(self, msg): self.current_odom = msg
    def scan_callback(self, msg): self.current_scan = msg
    def imu_callback(self, msg): self.current_imu = msg
    def plan_callback(self, msg): self.global_plan = msg

    def control_loop(self):
        # 检查所有必要数据是否都已收到
        if self.current_odom is None or self.current_scan is None or \
           self.current_imu is None or self.global_plan is None or \
           len(self.global_plan.poses) == 0:
            return

        # --- 核心逻辑：和数据处理脚本中几乎一样 ---
        # 1. 计算局部目标点
        current_pose = self.current_odom.pose.pose
        
        # 将Path消息中的poses转换成我们处理过的格式
        trajectory = [{'pos': np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]),
                       'ori': np.array([p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w])}
                      for p in self.global_plan.poses]

        # 找到路径上离小车当前位置最近的点
        current_pos_on_path_idx = np.argmin([np.linalg.norm(p['pos'][:2] - [current_pose.position.x, current_pose.position.y]) for p in trajectory])

        # 从这个最近点开始，向前寻找目标点
        lookahead_dist = 0.3 # 这是一个简化的方法，实际中可以用时间或更复杂算法
        target_idx = current_pos_on_path_idx
        while target_idx < len(trajectory) - 1:
            dist = np.linalg.norm(trajectory[target_idx]['pos'][:2] - trajectory[current_pos_on_path_idx]['pos'][:2])
            if dist >= lookahead_dist:
                break
            target_idx += 1
        
        future_pose = trajectory[target_idx]
        
        # 坐标系变换
        delta_pos_global = future_pose['pos'] - [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        current_q = np.array([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
        current_rotation = R.from_quat(current_q)
        goal_local = current_rotation.inv().apply(delta_pos_global)

        # 2. 准备模型输入
        scan_features = np.array(self.current_scan.ranges, dtype=np.float32)
        imu_features = np.array([self.current_imu.angular_velocity.z, self.current_imu.linear_acceleration.x], dtype=np.float32)
        goal_features = goal_local[:2].astype(np.float32)

        features = np.hstack([scan_features, imu_features, goal_features])
        input_tensor = torch.from_numpy(features).unsqueeze(0).to(self.device)

        # 3. 模型推理
        with torch.no_grad():
            action = self.model(input_tensor).squeeze().cpu().numpy()

        # 4. 发布控制指令
        cmd_msg = Twist()
        cmd_msg.linear.x = float(action[0])
        cmd_msg.angular.z = float(action[1])
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()