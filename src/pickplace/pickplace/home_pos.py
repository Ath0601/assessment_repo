#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class GoHomeNode(Node):
    def __init__(self):
        super().__init__('go_home_node')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/xarm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.send_home)

    def send_home(self):
        self.timer.cancel()

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        home = [
            0.0,                    
            -math.radians(45.0),      
            -math.radians(20.0),       
            0.0,                      
            0.0                        
        ]

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = home
        point.time_from_start.sec = 3

        traj.points.append(point)

        self.get_logger().info("Sending robot to HOME position...")
        self.pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = GoHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
