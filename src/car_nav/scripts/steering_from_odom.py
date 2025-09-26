#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32  # 假设舵机话题使用Int32，如需调整请修改
import math

class GimbalJointPublisher(Node):
    def __init__(self):
        super().__init__('gimbal_joint_publisher')

        # 参数设置
        self.declare_parameter('rate', 30.0)  # 发布频率(Hz)
        self.declare_parameter('servo_min', 0.0)   # 舵机最小值
        self.declare_parameter('servo_max', 180.0)  # 舵机最大值
        self.declare_parameter('joint_min', -1.57)  # 关节最小角度(rad)
        self.declare_parameter('joint_max', 1.57)   # 关节最大角度(rad)
        
        self.rate = float(self.get_parameter('rate').value)
        self.servo_min = float(self.get_parameter('servo_min').value)
        self.servo_max = float(self.get_parameter('servo_max').value)
        self.joint_min = float(self.get_parameter('joint_min').value)
        self.joint_max = float(self.get_parameter('joint_max').value)
        
        # 关节数据
        self.joints = ['jq1_Joint', 'jq2_Joint']
        self.joint_positions = {name: 0.0 for name in self.joints}
        
        # 订阅舵机控制话题
        self.s1_sub = self.create_subscription(
            Int32, '/servo_s1', self.s1_callback, 10)
        self.s2_sub = self.create_subscription(
            Int32, '/servo_s2', self.s2_callback, 10)

        # 发布关节状态
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
            
        # 定时器
        self.timer = self.create_timer(1.0/self.rate, self.publish_joint_states)
        
        self.get_logger().info('云台关节状态发布器已启动')
        
    def s1_callback(self, msg):
        # 将舵机值映射到关节角度
        servo_value = msg.data
        # 映射公式: 将舵机值从[servo_min, servo_max]映射到关节角度[joint_min, joint_max]
        joint_angle = self.map_value(servo_value, 
                                    self.servo_min, self.servo_max, 
                                    self.joint_min, self.joint_max)
        self.joint_positions['jq1_Joint'] = joint_angle
        
    def s2_callback(self, msg):
        # 同样映射第二个舵机值
        servo_value = msg.data
        joint_angle = self.map_value(servo_value, 
                                    self.servo_min, self.servo_max, 
                                    self.joint_min, self.joint_max)
        self.joint_positions['jq2_Joint'] = joint_angle
    
    def map_value(self, value, in_min, in_max, out_min, out_max):
        # 将值从一个范围映射到另一个范围
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def publish_joint_states(self):
        # 发布关节状态消息
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        msg.position = [self.joint_positions[name] for name in self.joints]
        msg.velocity = [0.0] * len(self.joints)  # 静态位置不需要速度
        msg.effort = []
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GimbalJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()