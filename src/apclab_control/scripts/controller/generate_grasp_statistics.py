#!/usr/bin/env python
import os
import sys
from math import pi
import yaml

import numpy as np
import pandas as pd
import rospkg
import tf

rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
from ur5_kin import UR5Kin


class GraspStats:
    """
    This script generates statistical results given the cube poses (w.r.t table)
        and arm joint angles.
    """
    def __init__(self, cube_pose_file, stats_file):
        config_metadata = self.load_metadata()
        self.cube2table_offset = float(config_metadata['cube2table_offset'])
        self.table_height = float(config_metadata['table_height'])
        self.grasp_offset = float(config_metadata['grasp_offset'])
        self.ur5_kin = UR5Kin()
        path = '/home/acrv-ur5/apcNet_dataset/exp_config'
        self.cube_poses_data = pd.read_csv(os.path.join(path, cube_pose_file))
        self.joint_stats = pd.read_csv(os.path.join(path, stats_file))
        self.sample_size = self.cube_poses_data.shape[0]

    def get_stats(self):
        raw_stats_stack = np.array([])
        for sample_idx in range(0, self.sample_size):
            start_idx = 20
            # if sample_idx in range(start_idx,start_idx+10) or
            # sample_idx in range(start_idx+30,start_idx+40):
            if not sample_idx in []:
                # print sample_idx
                cube_poses = self.cube_poses_data.iloc[sample_idx, :].values
                cube_6DoF_stack = np.array([])
                for cube_idx in range(0, 3):
                    cube_poses = cube_poses.reshape((3, -1))
                    cube_pose_temp = cube_poses[cube_idx, :]
                    cube_6DoF_pose = np.zeros([1, 6])
                    if cube_pose_temp[2] > 0:
                        cube_6DoF_pose[0, 0:3] = self.table_frame_to_world(
                            cube_pose_temp[0:3])
                        cube_6DoF_pose[0,
                        3:] = tf.transformations.euler_from_quaternion(
                            cube_pose_temp[3:])
                        if cube_6DoF_pose[0, 5] < 0:
                            cube_6DoF_pose[0, 5] = cube_6DoF_pose[0, 5] + 2 * pi
                        cube_6DoF_stack = np.append(cube_6DoF_stack,
                                                    cube_6DoF_pose)
                cube_6DoF_stack = np.around(cube_6DoF_stack.reshape((-1, 6)),
                                            decimals=5)
                joint_config = self.joint_stats.iloc[sample_idx, 2:8].values
                ee_pose_se3 = self.ur5_kin.get_ee_pose(
                    joint_config.reshape([6, 1]))
                ee_trans = (ee_pose_se3[0:3, 3]).reshape([3])
                ee_euler = tf.transformations.euler_from_matrix(
                    ee_pose_se3[0:3, 0:3])
                ee_6DoF_pose = np.append(ee_trans, ee_euler)
                if ee_6DoF_pose[3] < 0:
                    ee_6DoF_pose[3] = ee_6DoF_pose[3] + 2 * pi
                if ee_6DoF_pose[5] < 0:
                    ee_6DoF_pose[5] = ee_6DoF_pose[5] + 2 * pi
                cloest_target = self.find_closest_target(ee_trans,
                                                         cube_6DoF_stack)
                # print(ee_6DoF_pose, cloest_target)
                pose_diff = ee_6DoF_pose - cloest_target
                raw_stats_stack = np.append(raw_stats_stack, pose_diff)
        raw_stats_stack = raw_stats_stack.reshape((-1, 6))
        raw_stats_stack[:, 0:3] = raw_stats_stack[:, 0:3] * 100
        return np.around(raw_stats_stack, decimals=2)

    def get_mean_and_std(self, raw_data):
        data_mean = np.mean(raw_data, axis=0, keepdims=True)
        data_mean[0, 3:] = data_mean[0, 3:] * 57.295779513  # rad to degree
        data_std = np.std(raw_data, axis=0, keepdims=True)
        data_std[0, 3:] = data_std[0, 3:] * 57.295779513
        return np.around(data_mean, decimals=4), np.around(data_std, decimals=4)

    def find_closest_target(self, ee_trans, valide_targets_6DoF):
        n_valid_targets = valide_targets_6DoF.shape[0]
        min_distance = 100
        target_index = None
        for cube_idx in range(0, n_valid_targets):
            cube_xyz = valide_targets_6DoF[cube_idx, 0:3]
            distance_temp = np.linalg.norm(ee_trans - cube_xyz)
            if distance_temp < min_distance:
                min_distance = distance_temp
                target_index = cube_idx
        closest_target = valide_targets_6DoF[target_index, :]
        return closest_target

    def table_frame_to_world(self, xyz):
        xyz[1] = xyz[1] + 0.59
        xyz[2] = xyz[2] + self.table_height + self.grasp_offset
        return xyz

    def load_metadata(self):
        metadata_path = os.path.expanduser('~/apclab_dev/src/config.yml')
        with open(metadata_path, 'r') as config_file:
            try:
                config_metadata = yaml.load(config_file)
            except yaml.YAMLError as exc:
                print(exc)
        return config_metadata


def main(args):
    cube_pose_file = args[1]
    stats_file = args[2]
    stats_collector = GraspStats(cube_pose_file, stats_file)
    raw_stats = stats_collector.get_stats()
    print(raw_stats)
    data_mean, data_std = stats_collector.get_mean_and_std(raw_stats)
    print(data_mean)
    print(data_std)


if __name__ == '__main__':
    main(sys.argv)
