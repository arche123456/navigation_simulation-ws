#!/usr/bin/env python3
# init_robot_pose.py (修改版)

import rclpy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('init_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # 等待发布者建立连接
        time.sleep(2)
        
        subscription_count = self.publisher.get_subscription_count()
        self.get_logger().info(f"发布者已创建，当前订阅者数量: {subscription_count}")
        
        # 发布初始位置
        self.publish_initial_pose()
        
    def publish_initial_pose(self):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.w = 0.0
        
        # 设置协方差
        pose_msg.pose.covariance = [0.25] * 36  # 简化设置
        pose_msg.pose.covariance[0] = 0.25  # x
        pose_msg.pose.covariance[7] = 0.25  # y
        pose_msg.pose.covariance[35] = 0.06853891909122467  # yaw
        
        self.publisher.publish(pose_msg)
        self.get_logger().info('✅ 初始位置消息已发布')
        self.get_logger().info(f'消息内容: position=({pose_msg.pose.pose.position.x}, {pose_msg.pose.pose.position.y})')
        
        # 短暂延迟确保消息发送
        time.sleep(0.5)

def main():
    rclpy.init()
    
    try:
        node = InitialPosePublisher()
        # 短暂运行确保消息处理
        rclpy.spin_once(node, timeout_sec=1.0)
        node.destroy_node()
    except Exception as e:
        print(f"错误: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()