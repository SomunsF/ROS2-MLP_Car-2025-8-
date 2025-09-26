#!/usr/bin/env python3
"""
功能解释：
这是一个里程计桥接节点，主要功能包括：
1. 订阅原始里程计数据（/odom_raw话题）
2. 将里程计数据转换为标准的nav_msgs/Odometry格式
3. 发布TF变换：odom -> base_footprint
4. 重新发布修正后的里程计数据到/odom话题
5. 确保坐标系名称符合Nav2导航栈的要求

这个节点是连接硬件里程计数据和导航系统的重要桥梁。
"""

import rclpy  # 导入ROS2 Python客户端库
from rclpy.node import Node  # 导入节点基类
import tf2_ros  # 导入TF2变换库
from geometry_msgs.msg import TransformStamped  # 导入变换消息类型
from nav_msgs.msg import Odometry  # 导入里程计消息类型

class OdomBridge(Node):  # 定义里程计桥接节点类，继承自Node
    def __init__(self):  # 初始化函数
        super().__init__('odom_bridge')  # 调用父类初始化函数，设置节点名称为'odom_bridge'
        
        # TF broadcaster  # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)  # 创建TF广播器，用于发布坐标变换
        
        # Subscribe to raw odom data  # 订阅原始里程计数据
        self.odom_sub = self.create_subscription(  # 创建订阅者
            Odometry,  # 消息类型：里程计
            '/odom_raw',  # 话题名称：原始里程计数据
            self.odom_callback,  # 回调函数
            100  # 队列大小
        )
        
        # Publish corrected odom data  # 发布修正后的里程计数据
        self.odom_pub = self.create_publisher(Odometry, '/odom', 100)  # 创建发布者，发布到/odom话题
        
        self.get_logger().info('Odom Bridge node started')  # 打印节点启动信息
        
    def odom_callback(self, msg):  # 里程计数据回调函数
        """Convert odom_frame to odom frame and publish TF"""  # 将odom_frame转换为odom坐标系并发布TF
        
        # Publish TF: odom -> base_footprint  # 发布TF变换：odom -> base_footprint
        transform = TransformStamped()  # 创建变换消息
        transform.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳为当前时间
        transform.header.frame_id = 'odom'  # 设置父坐标系为odom
        transform.child_frame_id = 'base_footprint'  # 设置子坐标系为base_footprint
        
        # Copy pose from message  # 从消息中复制位姿信息
        transform.transform.translation.x = msg.pose.pose.position.x  # 复制x坐标
        transform.transform.translation.y = msg.pose.pose.position.y  # 复制y坐标
        transform.transform.translation.z = msg.pose.pose.position.z  # 复制z坐标
        
        transform.transform.rotation.x = msg.pose.pose.orientation.x  # 复制四元数x分量
        transform.transform.rotation.y = msg.pose.pose.orientation.y  # 复制四元数y分量
        transform.transform.rotation.z = msg.pose.pose.orientation.z  # 复制四元数z分量
        transform.transform.rotation.w = msg.pose.pose.orientation.w  # 复制四元数w分量
        
        self.tf_broadcaster.sendTransform(transform)  # 广播TF变换
        
        # Publish corrected odom message  # 发布修正后的里程计消息
        corrected_odom = Odometry()  # 创建新的里程计消息
        corrected_odom.header = msg.header  # 复制消息头
        corrected_odom.header.frame_id = 'odom'  # Change frame_id to odom  # 将坐标系ID改为odom
        corrected_odom.child_frame_id = 'base_footprint'  # 设置子坐标系为base_footprint
        corrected_odom.pose = msg.pose  # 复制位姿信息
        corrected_odom.twist = msg.twist  # 复制速度信息
        
        self.odom_pub.publish(corrected_odom)  # 发布修正后的里程计消息

def main(args=None):  # 主函数
    rclpy.init(args=args)  # 初始化ROS2
    odom_bridge = OdomBridge()  # 创建里程计桥接节点实例
    
    try:  # 尝试运行
        rclpy.spin(odom_bridge)  # 保持节点运行，处理回调函数
    except KeyboardInterrupt:  # 捕获键盘中断异常
        pass  # 忽略异常
    
    odom_bridge.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':  # 如果作为主程序运行
    main()  # 调用主函数
