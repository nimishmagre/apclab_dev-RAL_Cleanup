#!/usr/bin/env python
import random
from math import pi

import numpy as np
import rospy
import tf
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState

CUBE_LENGTH = 0.08


def generate_poses(p=0.3):
    valid_poses = False
    while not valid_poses:
        x_buff = np.around(np.random.uniform(-0.5, 0.5, (1, 3)), decimals=3)
        y_buff = np.around(np.random.uniform(-0.15, 0.25, (1, 3)), decimals=3)
        xy_buff = np.vstack((x_buff, y_buff))
        d1 = np.linalg.norm(xy_buff[:, 0] - xy_buff[:, 1])
        d2 = np.linalg.norm(xy_buff[:, 0] - xy_buff[:, 2])
        d3 = np.linalg.norm(xy_buff[:, 1] - xy_buff[:, 2])
        if d1 > 1.5 * CUBE_LENGTH and d2 > 1.5 * CUBE_LENGTH and d3 > 1.5 * CUBE_LENGTH:
            valid_poses = True
    print(xy_buff)
    target_poses = []
    for i in range(0, 3):
        target = LinkState()
        target.link_name = 'cube' + str(i + 1) + '_link'
        target.reference_frame = "table_surface_link"
        x = xy_buff[0, i]
        y = xy_buff[1, i]
        prob_token = random.uniform(0, 1)
        if prob_token < p:
            z = -0.2
        else:
            z = CUBE_LENGTH / 4 + 0.0127
        yaw = round(random.uniform(pi / 2, 3 * pi / 2), 5)
        quaternion = tf.transformations.quaternion_from_euler(pi, 0, yaw)
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]
        target_poses.append(target)
    return target_poses


def move_cube(target_poses):
    try:
        set_cube_position = rospy.ServiceProxy(
            '/gazebo/set_link_state', SetLinkState)
        for target in target_poses:
            move_status = set_cube_position(target)
    except rospy.ServiceException, e:
        print "Service call failed: %s"


def move_cubes_sim():
    rospy.wait_for_service('/gazebo/set_link_state')
    target_poses = generate_poses()
    move_cube(target_poses)


if __name__ == '__main__':
    rospy.init_node("move_cube_rand", anonymous=True)
    move_cubes_sim()
