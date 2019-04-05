#!/usr/bin/env python

import sys
from math import pi

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def arm2home_sim():
    """
    Moves arm to home position in the simulation
    """
    joint_command_pub = rospy.Publisher("/arm_controller/command",
                                        JointTrajectory, queue_size=10)
    finger_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory,
                                 queue_size=10)
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joint_traj = JointTrajectory()
    joint_traj.joint_names = joint_names
    joint_traj.points = [JointTrajectoryPoint(
        positions=[3 * pi / 4, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0],
        velocities=[0] * 6,
        time_from_start=rospy.Duration(0.1))]
    finger_names = ['finger_joint', 'left_inner_finger_joint',
                    'left_inner_knuckle_joint', 'right_inner_finger_joint',
                    'right_inner_knuckle_joint', 'right_outer_knuckle_joint']
    close_level = 0
    finger_angle = close_level * (0.77 / 5)
    finger_traj = JointTrajectory()
    # finger_traj.header = msg.header
    finger_traj.joint_names = finger_names
    finger_joint_config = np.array(
        [1.0, 1.0, -1.0, 1.0, -1.0, -1.0]) * finger_angle
    finger_traj.points = [JointTrajectoryPoint(positions=finger_joint_config,
                                               velocities=[0] * 6,
                                               time_from_start=rospy.Duration(
                                                   3.0))]
    rospy.sleep(0.01)

    for i in range(100):
        joint_command_pub.publish(joint_traj)
        finger_pub.publish(finger_traj)
        rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node("arm_home", anonymous=True)
    arm2home_sim()
    sys.exit("Arm is Home, \"process has died\" warning is normal")
