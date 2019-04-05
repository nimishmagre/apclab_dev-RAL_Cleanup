#!/usr/bin/env python
import sys
import os
import random
from math import pi, sin, cos
import math
import numpy as np
import tf
from gazebo_msgs.msg import LinkState
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import GetLinkState
import rospkg
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
sys.path.append(os.path.join(rospack.get_path('apclab_dset_collector'), 'scripts'))
from pose_msg_converter import *
from ur5_kin import UR5Kin
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class Ur5KinCalibrator():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.current_joint_config = None
        self.joint_config_sub = rospy.Subscriber("/arm_controller/state",
                    JointTrajectoryControllerState, self.joint_config_callback)
        self.ur5_kin = UR5Kin()

    def joint_config_callback(self, msg):
        self.current_joint_config = np.asarray(msg.actual.positions).reshape((6,1))

    def compare_results(self):
        analyt_ee_pose = self.ur5_kin.get_ee_pose(self.current_joint_config)
        actual_ee_pose = pose_msg2se3(self.group.get_current_pose())
        print('Computed:')
        print(analyt_ee_pose)
        print('GT')
        print(actual_ee_pose)

def main(args):
    rospy.init_node("ur5_kin_calibration", anonymous=True)
    ur5_kin_calibrator = Ur5KinCalibrator()
    rospy.sleep(0.5)
    ur5_kin_calibrator.compare_results()
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print ("Shutting down...")

if __name__ == "__main__":
    main(sys.argv)
