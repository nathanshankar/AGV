#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('arm_trajectory_publisher')

        # Create a publisher for the joint trajectory
        publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Create the JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'finger_1_joint', 'finger_2_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.613, 2.642, 0.642, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 5000

        trajectory_msg.points.append(point)

        # Publish the message once
        self.get_logger().info('Publishing joint trajectory...')
        publisher.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTrajectoryPublisher()
    rclpy.spin_once(node)

if __name__ == '__main__':
    main()
