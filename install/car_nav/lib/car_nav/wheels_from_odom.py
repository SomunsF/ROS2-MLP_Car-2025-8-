#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class WheelsFromOdom(Node):
    def __init__(self):
        super().__init__('wheels_from_odom')

        # 可调参数：轮半径、轮距、发布频率
        self.declare_parameter('wheel_radius', 0.015)   # 你的轮半径 m
        self.declare_parameter('wheel_track', 0.135)    # 左右轮心距 m（按实测改）
        self.declare_parameter('rate', 30.0)            # 发布 Hz

        self.R = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_track').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)

        # 当前车体速度（来自 /odom_raw.twist）
        self.vx = 0.0
        self.wz = 0.0

        # 关节累计角度（rad）
        # 关节顺序按你给的：['zh_Joint','zq_Joint','yh_Joint','yq_Joint']
        self.joints = ['zh_Joint', 'zq_Joint', 'yh_Joint', 'yq_Joint']
        self.theta = {name: 0.0 for name in self.joints}

        # 轴向补偿：左侧 +1（zh/zq），右侧 -1（yh/yq），因为右侧 URDF 轴是 0 -1 0
        self.sign = {'zh_Joint': +1, 'zq_Joint': +1, 'yh_Joint': -1, 'yq_Joint': -1}

        self.sub = self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().info(f'wheels_from_odom running: R={self.R:.3f}m L={self.L:.3f}m dt={self.dt:.3f}s')

    def odom_cb(self, msg: Odometry):
        # 直接用 twist（更鲁棒，避免位姿离散跳变）
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

    def tick(self):
        # 差分模型：左右侧轮角速度（rad/s）
        wl = (self.vx - self.wz * self.L / 2.0) / self.R
        wr = (self.vx + self.wz * self.L / 2.0) / self.R

        # 按轴向补偿积分角度
        self.theta['zh_Joint'] += self.sign['zh_Joint'] * wl * self.dt  # 左后
        self.theta['zq_Joint'] += self.sign['zq_Joint'] * wl * self.dt  # 左前
        self.theta['yh_Joint'] += self.sign['yh_Joint'] * wr * self.dt  # 右后
        self.theta['yq_Joint'] += self.sign['yq_Joint'] * wr * self.dt  # 右前

        # 发布 JointState（velocity 同样按 sign 处理，方便可视化）
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joints
        js.position = [self.theta[n] for n in self.joints]
        js.velocity = [
            self.sign['zh_Joint'] * wl,
            self.sign['zq_Joint'] * wl,
            self.sign['yh_Joint'] * wr,
            self.sign['yq_Joint'] * wr,
        ]
        js.effort = []
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = WheelsFromOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
