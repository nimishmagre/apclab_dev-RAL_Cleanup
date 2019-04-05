#!/usr/bin/env python
import os
import sys
import random
import math
import csv
import glob
import cv2
import pandas as pd
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
# ros packages
import rospy
import rospkg
import geometry_msgs.msg
from sensor_msgs.msg import Image
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState

bridge = CvBridge()

class SampleCollector():
    def __init__(self, config_file_name, dataset_name):
        path = os.path.expanduser('~/apcNet_dataset/exp_config')
        self.config_file_path = os.path.join(path, config_file_name)
        self.dataset_path = os.path.join(os.path.expanduser('~/apcNet_dataset/exp_dataset'),
                                                                    dataset_name)
        # get configs from config file
        # cube is specified wrt world
        self.start_idx = 0
        self.cv2_img = None
        self.current_joint_config = None
        self.targets_reached = False
        self.dataset_prefix = dataset_name+'_'
        # ros sub and pub
        self.image_sub = rospy.Subscriber("/sim_camera/image_raw",
                                            Image, self.image_callback)
        self.cube_pose_file = pd.read_csv(self.config_file_path)

        # check file path, if path exists, no hearder needed
        self.check_file_path()


    def read_cube_pose_file(self, sample_idx):
        cube_poses = self.cube_pose_file.iloc[sample_idx, :].values
        # print(cube_poses)
        cube_poses = cube_poses.reshape((3, -1))
        target_poses = []
        for i in range(0, 3):
            target = LinkState()
            target.link_name = 'cube'+str(i+1)+'_link'
            target.reference_frame = "table_surface_link"
            target.pose.position.x = cube_poses[i, 0]
            target.pose.position.y = cube_poses[i, 1]
            target.pose.position.z = cube_poses[i, 2]
            target.pose.orientation.x = cube_poses[i, 3]
            target.pose.orientation.y = cube_poses[i, 4]
            target.pose.orientation.z = cube_poses[i, 5]
            target.pose.orientation.w = cube_poses[i, 6]
            target_poses.append(target)
        return target_poses

    def collecting_samples(self):
        rospy.wait_for_service('/gazebo/set_link_state')
        n_samples = self.cube_pose_file.shape[0]
        for sample_idx in range(0, n_samples):
            target_poses = self.read_cube_pose_file(sample_idx)
            self.move_cube(target_poses)
            rospy.sleep(0.2)
            file_name = str(sample_idx)+'.jpeg'
            self.save_image(file_name)

    def move_cube(self, target_poses):
        try:
            set_cube_position = rospy.ServiceProxy(
                '/gazebo/set_link_state', SetLinkState)
            for target in target_poses:
                move_status = set_cube_position(target)
        except rospy.ServiceException, e:
            print "Service call failed: %s"

    #############
    # Tool Box  #
    #############
    def save_image(self, image_name):
        image_full_path = os.path.join(self.dataset_path, image_name)
        cv2.imwrite(image_full_path, self.cv2_img)
        print ("Image " + image_name + " Captured \n")

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)

    def check_file_path(self):
        if os.path.exists(self.dataset_path):
            self.need_csv_header = False
            pattern = "*.jpeg"
            self.start_idx = len(glob.glob1(self.dataset_path, pattern))
            print ('\n \nResume from %dth sample...' %self.start_idx)
        else:
            os.makedirs(self.dataset_path)

    def get_image_filename(self):
        image_format_suffix = ".jpeg"
        pattern = "*" + image_format_suffix
        img_list = glob.glob1(self.dataset_path, pattern)
        idx = len(img_list)
        zero_padded_idx = "%.5d" % (idx)
        filename = self.dataset_prefix + \
            zero_padded_idx + image_format_suffix
        return filename


def main(args):
    rospy.init_node("sample_collector", anonymous=True)
    config_file_name = args[1]
    dataset_name = args[2]
    sample_collector = SampleCollector(config_file_name, dataset_name)
    sample_collector.collecting_samples()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")

if __name__ == "__main__":
    main(sys.argv)
