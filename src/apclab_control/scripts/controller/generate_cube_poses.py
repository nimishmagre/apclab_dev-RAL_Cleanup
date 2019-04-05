#!/usr/bin/env python
import csv
import os
import random
import sys
from math import pi

import numpy as np
import rospy
import tf
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState

CUBE_LENGTH = 0.08


def generate_poses(n_cubes):
    valid_poses = False
    while not valid_poses:
        x_buff = np.around(np.random.uniform(-0.45, 0.45, (1, 3)), decimals=3)
        y_buff = np.around(np.random.uniform(-0.15, 0.23, (1, 3)), decimals=3)
        xy_buff = np.vstack((x_buff, y_buff))
        d1 = np.linalg.norm(xy_buff[:, 0] - xy_buff[:, 1])
        d2 = np.linalg.norm(xy_buff[:, 0] - xy_buff[:, 2])
        d3 = np.linalg.norm(xy_buff[:, 1] - xy_buff[:, 2])
        if d1 > 1.5 * CUBE_LENGTH and d2 > 1.5 * CUBE_LENGTH and \
                d3 > 1.5 * CUBE_LENGTH:
            valid_poses = True
    new_row = []
    for i in range(0, 3):
        target = LinkState()
        target.link_name = 'cube' + str(i + 1) + '_link'
        target.reference_frame = "table_surface_link"
        x = xy_buff[0, i]
        y = xy_buff[1, i]
        if i < n_cubes:
            z = CUBE_LENGTH / 4 + 0.0127
        else:
            z = -0.2
        yaw = round(random.uniform((pi * 3) / 5, (8 * pi / 5)), 5)
        quaternion = tf.transformations.quaternion_from_euler(pi, 0, yaw)
        new_row = new_row + [x, y, z] + list(quaternion)
    return new_row


def move_cube(target_poses):
    try:
        set_cube_position = rospy.ServiceProxy(
            '/gazebo/set_link_state', SetLinkState)
        for target in target_poses:
            move_status = set_cube_position(target)
    except rospy.ServiceException, e:
        print "Service call failed: %s"


def generate_pose_file(n_samples_each, file_name):
    token = np.arange(1, 4)
    sampling_sequence = token.repeat(n_samples_each)
    # print(sampling_sequence)

    file_path = os.path.join(os.path.expanduser('~/apcNet_dataset/exp_config/'),
                             file_name)
    with open(file_path, "a") as pose_file:
        writer = csv.writer(pose_file)
        for n_cubes in sampling_sequence:
            new_row = generate_poses(n_cubes)
            writer.writerow(new_row)


if __name__ == '__main__':
    rospy.init_node("generate_cube_pose_file", anonymous=True)
    generate_pose_file(10, sys.argv[1])
