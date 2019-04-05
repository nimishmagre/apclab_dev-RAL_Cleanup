#!/usr/bin/env python
import os
import sys
import random
import math
import csv
import glob
import cv2
import time
import pandas as pd
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
# ros packages
import rospy
import rospkg
import message_filters
import geometry_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from gazebo_msgs.srv import GetLinkState
# my scripts
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
from ur5_kin import UR5Kin
from pose_msg_converter import *

bridge = CvBridge()

class SampleCollector():
    def __init__(self, config_file_name, dataset_name):
        path = os.path.expanduser('~/apcNet_dataset/scene_config')
        self.config_file_path = os.path.join(path, config_file_name)
        self.dataset_path = os.path.join(os.path.expanduser('~/apcNet_dataset/dataset'),
                                                                    dataset_name)
        # get configs from config file
        # cube is specified wrt world
        self.start_idx = 0
        self.cv2_img_right = None
	self.cv2_img_left = None
        self.current_joint_config = None
        self.targets_reached = False
        self.dataset_prefix = dataset_name+'_'
        # ros sub and pub
        #def callback(self,msg,msg1):
	    #self.cv2_img_right = bridge.imgmsg_to_cv2(msg, "bgr8")
	    #self.cv2_img_left = bridge.imgmsg_to_cv2(msg1, "bgr8")

        #self.right_image_sub = rospy.Subscriber("/multilense_sl/camera_pg/right/image_raw",
                                            #Image,self.right_image_callback)
        #self.left_image_sub = rospy.Subscriber("/multilense_sl/camera_pg/left/image_raw",
					    #Image,self.left_image_callback)
	self.right_image_sub = message_filters.Subscriber("/multilense_sl/camera_pg/right/image_raw",
					    Image)
	self.left_image_sub = message_filters.Subscriber("/multilense_sl/camera_pg/left/image_raw",
					    Image)
        #self.ts = message_filters.TimeSynchronizer([self.right_image_sub, self.left_image_sub], 10)
	#self.ts.registerCallback(self.callback)
	#rospy.spin()

        self.joint_config_sub = rospy.Subscriber("/arm_controller/state",
                    JointTrajectoryControllerState, self.joint_config_callback)
        self.joint_command_pub = rospy.Publisher("/arm_controller/command",
                                            JointTrajectory, queue_size=10)

	self.ts = message_filters.TimeSynchronizer([self.right_image_sub, self.left_image_sub], 10)
	self.ts.registerCallback(self.callback)
	#rospy.spin()

    def callback(self,Image,Image1):
	self.right_image_callback(Image)
	self.left_image_callback(Image1)

	#r = rospy.Rate(16) # 16hz
        rospy.loginfo('syncing')
        # csv header
        self.need_csv_header = True
        cube_pose_header = ["cube1_x", "cube1_y","cube1_z", "cube1_a",
                                "cube1_b", "cube1_c", "cube1_d",
                                "cube2_x", "cube2_y","cube2_z", "cube2_a",
                                "cube2_b", "cube2_c", "cube2_d",
                                "cube3_x", "cube3_y","cube3_z", "cube3_a",
                                "cube3_b", "cube3_c", "cube3_d"]
        camera_pose_header = ["cam_x", "cam_y", "cam_z",
                                    "cam_a", "cam_b", "cam_c", "cam_d"]
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.csv_header = ["right_image_name"] + ["left_image_name"] + self.joint_names\
            + cube_pose_header + camera_pose_header
        # check file path, if path exists, no hearder needed
        self.check_file_path()


    def collecting_samples(self):
        cube1_quats, cube2_quats, cube3_quats, joint_configs = \
                                    self.get_cube_and_joint_config(self.config_file_path)
        csv_full_path = os.path.join(self.dataset_path, "armbot_dataset_3cubes.csv")
        with open(csv_full_path, "a") as file:
            writer = csv.writer(file)
            self.need_csv_header = True
            if self.need_csv_header:
                    writer.writerow(self.csv_header)
                    self.need_csv_header = False
            sample_size = len(joint_configs)
            if self.start_idx == sample_size:
                print ('Dataset already collected, shutting down...')
                rospy.signal_shutdown('Quit')
            elif self.start_idx > sample_size:
                print('Loading different scene configuration file...')
                self.start_idx = 0
            for sample_idx in range(self.start_idx, sample_size):
                print ('Processing Sample %d' % sample_idx)
                cube1_target_pose = cube1_quats[sample_idx,:]
                cube2_target_pose = cube2_quats[sample_idx,:]
                cube3_target_pose = cube3_quats[sample_idx,:]
                joint_target = joint_configs[sample_idx,:]
                joint_msg = self.vec2joint_traj_msg(joint_target)
                cube1_at_target = self.move_cube_to_target('cube1_link', cube1_target_pose)
                cube2_at_target = self.move_cube_to_target('cube2_link', cube2_target_pose)
                cube3_at_target = self.move_cube_to_target('cube3_link', cube3_target_pose)
                cubes_at_target = cube1_at_target and cube2_at_target and cube3_at_target
                while not self.targets_reached:
                    self.joint_command_pub.publish(joint_msg)
                    rospy.sleep(0.02)
                    self.targets_reached = cubes_at_target and self.arm_reached_target(
                            self.current_joint_config, joint_target)
                rospy.sleep(0.1)

                if self.targets_reached:
                    right_image_name  = self.get_right_image_filename()
                    self.save_image_right(right_image_name)
		    left_image_name  = self.get_left_image_filename()
                    self.save_image_left(left_image_name)
                    new_row = self.state_2_csv_row(right_image_name, left_image_name)
                    writer.writerow(new_row)
                    self.targets_reached = False

                if sample_idx == sample_size-1:
                    print ('Samples are all collected, shutting down...')
                    rospy.signal_shutdown('Quit')

    def state_2_csv_row(self, right_image_name, left_image_name):
        camera_pose = list(self.get_link_pose('real_sense_link', 'world'))
        cube1_pose = list(self.get_link_pose('cube1_link', 'world'))
        cube2_pose = list(self.get_link_pose('cube2_link', 'world'))
        cube3_pose = list(self.get_link_pose('cube3_link', 'world'))

        current_joint_config = list(self.current_joint_config)
        new_row = [right_image_name] + [left_image_name] + current_joint_config + cube1_pose \
                                + cube3_pose + cube2_pose + camera_pose
        return new_row


    def move_cube_to_target(self, link_name, target_pose):
        rospy.wait_for_service('/gazebo/set_link_state')
        target = LinkState()
        target.link_name = link_name
        target.reference_frame = "link"
        target.pose.position.x = target_pose[0]
        target.pose.position.y = target_pose[1]
        target.pose.position.z = target_pose[2]
        target.pose.orientation.x = target_pose[3]
        target.pose.orientation.y = target_pose[4]
        target.pose.orientation.z = target_pose[5]
        target.pose.orientation.w = target_pose[6]
        try:
            set_cube_position = rospy.ServiceProxy(
                '/gazebo/set_link_state', SetLinkState)
            move_status = set_cube_position(target)
            return move_status.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"

    def arm_reached_target(self, joint_config, joint_target):
        joint_distance = np.linalg.norm(joint_config-joint_target)
        joint_arrive = joint_distance<1e-2
        if joint_arrive:
            targets_reached = True
        else:
            targets_reached = False
        return targets_reached

    #############
    # Tool Box  #
    #############
    def save_image_right(self, right_image_name):
        image_full_path = os.path.join(self.dataset_path, right_image_name)
        cv2.imwrite(image_full_path, self.cv2_img_right)
        print ("Right Image " + right_image_name + " Captured \n")

    def save_image_left(self, left_image_name):
        image_full_path = os.path.join(self.dataset_path, left_image_name)
        cv2.imwrite(image_full_path, self.cv2_img_left)
        print ("Left Image " + left_image_name + " Captured \n")

    def get_link_pose(self, link_name, reference_frame):
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            current_link_state = rospy.ServiceProxy(
                '/gazebo/get_link_state', GetLinkState)
            link_state = current_link_state(link_name, reference_frame)
            link_pose = link_state.link_state
            return pose_msg2vec(link_pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"

    def get_cube_and_joint_config(self, config_file_path):
        config_data = pd.read_csv(config_file_path)
        joint_configs = config_data.iloc[:, 0:6].values
        joint_configs = joint_configs.astype(np.float)
        cube1_quats = config_data.iloc[:, 6:13].values
        cube1_quats = cube1_quats.astype(np.float)
        cube2_quats = config_data.iloc[:, 13:20].values
        cube2_quats = cube2_quats.astype(np.float)
        cube3_quats = config_data.iloc[:, 20:27].values
        cube3_quats = cube3_quats.astype(np.float)
        return cube1_quats, cube2_quats, cube3_quats, joint_configs

    def joint_config_callback(self, msg):
        self.current_joint_config = msg.actual.positions

    def right_image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img_right = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
    def left_image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img_left = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)

    def vec2joint_traj_msg(self, joint_config_vec):
        joint_traj = JointTrajectory()
        joint_traj.joint_names = self.joint_names
        joint_traj.points = [JointTrajectoryPoint(positions=joint_config_vec,
                                                  velocities=[0] * 6,
                                    time_from_start=rospy.Duration(0.1))]
        return joint_traj

    def check_file_path(self):
        if os.path.exists(self.dataset_path):
            self.need_csv_header = False
            pattern = "*.jpeg"
            self.start_idx = len(glob.glob1(self.dataset_path, pattern))
            print ('\n \nResume from %dth sample...' %self.start_idx)
        else:
            os.makedirs(self.dataset_path)

    def get_right_image_filename(self):
        image_format_suffix = "right.jpeg"
        pattern = "*" + image_format_suffix
        img_list = glob.glob1(self.dataset_path, pattern)
        idx = len(img_list)
        zero_padded_idx = "%.5d" % (idx)
        filename = self.dataset_prefix+ \
            zero_padded_idx + image_format_suffix
        return filename

    def get_left_image_filename(self):
        image_format_suffix = "left.jpeg"
        pattern = "*" + image_format_suffix
        img_list = glob.glob1(self.dataset_path, pattern)
        idx = len(img_list)
        zero_padded_idx = "%.5d" % (idx)
        filename = self.dataset_prefix+ \
            zero_padded_idx + image_format_suffix
        return filename


def main(args):
    rospy.init_node("sample_collector", anonymous=True)
    config_file_name = 'b.csv' #args[1]
    dataset_name = 'dset' #args[2]
    sample_collector = SampleCollector(config_file_name, dataset_name)
    sample_collector.collecting_samples()
    try:
        rospy.spin()
    except KeyboardInterrupt: 
        print ("Shutting down ROS Image feature detector module")

if __name__ == "__main__":
    main(sys.argv)

