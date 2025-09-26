#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribe to raw odom data
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            10
        )
        
        # Publish corrected odom data
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.get_logger().info('Odom Bridge node started')
        
    def odom_callback(self, msg):
        """Convert odom_frame to odom frame and publish TF"""
        
        # Publish TF: odom -> base_footprint
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        
        # Copy pose from message
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(transform)
        
        # Publish corrected odom message
        corrected_odom = Odometry()
        corrected_odom.header = msg.header
        corrected_odom.header.frame_id = 'odom'  # Change frame_id to odom
        corrected_odom.child_frame_id = 'base_footprint'
        corrected_odom.pose = msg.pose
        corrected_odom.twist = msg.twist
        
        self.odom_pub.publish(corrected_odom)

def main(args=None):
    rclpy.init(args=args)
    odom_bridge = OdomBridge()
    
    try:
        rclpy.spin(odom_bridge)
    except KeyboardInterrupt:
        pass
    
    odom_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
