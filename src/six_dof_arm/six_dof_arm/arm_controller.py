#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Create timer for publishing joint states
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize joint states
        self.joint_state = JointState()
        self.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state.position = [1.2, -2.4, 2.0, 1.0, -1.0, 0.0]
        
        self.get_logger().info('Arm controller node initialized')

    def timer_callback(self):
        # Update joint states
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)

    def move_joint(self, joint_index, position):
        if 0 <= joint_index < len(self.joint_state.position):
            self.joint_state.position[joint_index] = position
            self.get_logger().info(f'Moving joint {joint_index} to position {position}')
        else:
            self.get_logger().error('Invalid joint index')

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
