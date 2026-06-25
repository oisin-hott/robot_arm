#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JoyToArm(Node):
    def __init__(self):
        super().__init__('joy_to_arm')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.callback,
            10
        )

        self.positions = [0.0] * 6

    def callback(self, msg):
        if len(msg.axes) < 2:
            return

        self.positions[0] += msg.axes[0] * 0.02
        self.positions[1] += msg.axes[1] * 0.02

        traj = JointTrajectory()
        traj.joint_names = [
            'joint_1','joint_2','joint_3',
            'joint_4','joint_5','joint_6'
        ]

        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1

        traj.points.append(point)

        self.pub.publish(traj)


def main():
    rclpy.init()
    node = JoyToArm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()