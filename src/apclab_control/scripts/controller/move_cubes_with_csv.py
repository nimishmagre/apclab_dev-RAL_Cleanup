#!/usr/bin/env python
import sys

import pandas as pd
import rospy
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState


def read_cube_pose_file(csv_path, sample_idx):
    pose_file = pd.read_csv(csv_path)
    print(pose_file.shape)
    cube_poses = pose_file.iloc[sample_idx, :].values
    # print(cube_poses)
    cube_poses = cube_poses.reshape((3, -1))
    target_poses = []
    for i in range(0, 3):
        target = LinkState()
        target.link_name = 'cube' + str(i + 1) + '_link'
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


def move_cube(target_poses):
    try:
        set_cube_position = rospy.ServiceProxy(
            '/gazebo/set_link_state', SetLinkState)
        for target in target_poses:
            move_status = set_cube_position(target)
    except rospy.ServiceException, e:
        print "Service call failed: %s"


def move_cubes_sim(args):
    file_name = args[1]
    sample_idx = int(args[2])
    rospy.wait_for_service('/gazebo/set_link_state')
    target_poses = read_cube_pose_file(file_name, sample_idx)
    move_cube(target_poses)


if __name__ == '__main__':
    rospy.init_node("move_cube_rand", anonymous=True)
    move_cubes_sim(sys.argv)
