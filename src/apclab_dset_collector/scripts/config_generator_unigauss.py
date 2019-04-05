#!/usr/bin/env python
import os
import sys
import random
from math import pi, cos, sin, asin
from tqdm import tqdm
import csv
import yaml
import tf
import numpy as np
import rospkg
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
from ur5_kin import UR5Kin
from search_low_lyap import LowLyapSearch


class ConfigGenerator():
    def __init__(self, cube_sample_size, file_name, dataset_focus):
        # Parameter Initialisation
        config_metadata = self.load_metadata()
        self.cube_sample_size = cube_sample_size
        self.ur5_kin = UR5Kin()
        self.table_height = float(config_metadata['table_height'])
        self.cube_length = float(config_metadata['cube_length'])
        self.cube2table_offset = float(config_metadata['cube2table_offset'])
        self.joint_lower_limits = np.array([[pi/4], [-pi/2], [0], [-pi],[-pi], [-pi/2]])
        self.joint_upper_limits = np.array([[5*pi/4], [-pi/6], [pi/2], [0], [0], [pi]])
        self.cube_pose_header = ["cube1_x", "cube1_y","cube1_z", "cube1_a",
                                "cube1_b", "cube1_c", "cube1_d",
                                "cube2_x", "cube2_y","cube2_z", "cube2_a",
                                "cube2_b", "cube2_c", "cube2_d",
                                "cube3_x", "cube3_y","cube3_z", "cube3_a",
                                "cube3_b", "cube3_c", "cube3_d",]
        self.joint_names = ['s_pan','s_lift','elbow','w1','w2','w3']
        self.path = os.path.expanduser('~/apcNet_dataset/scene_config')
        self.csv_path = os.path.join(self.path, file_name)
        self.check_file_path()
        self.csv_needs_header = True
        self.dataset_focus = dataset_focus #local or global

    def generate_scene_config(self):
        header = self.joint_names + self.cube_pose_header
        with open(self.csv_path, "a") as file:
            writer = csv.writer(file)
            if self.csv_needs_header:
                writer.writerow(header)
                self.csv_needs_header = False
            print ("Generating Scene Configurations...")
            total_sample_size = 0
            for i in tqdm(range(self.cube_sample_size)):
                cube_poses_quat = self.get_random_cube_pose()
                for j in range (0, cube_poses_quat.shape[0]):
                    # h2t_6D_poses = self.get_h2t_6D_poses_dense(15)
                    if self.dataset_focus == 'global':
                        h2t_radius = self.get_random_radius_vec(0, 0.25, 0.08, 0.6, 5)
                        h2t_6D_poses = self.get_h2t_6D_poses(h2t_radius, 20)
                    elif self.dataset_focus == 'local':
                        h2t_radius = self.get_random_radius_vec(0.08, 0.15, 0.08, 0.20, 10)
                        h2t_6D_poses = self.get_h2t_6D_poses(h2t_radius, 20)
                    else:
                        sys.exit("The Dataset Focus Needs to Be Either \'global\' or \'local\'")

                    if cube_poses_quat[j, 2]>self.table_height:  # if the cube is visible
                        X_T = self.quat2mat(cube_poses_quat[j,:])
                        joint_config_stack = self.get_joint_config_stack(h2t_6D_poses, X_T)
                        if joint_config_stack is not None:
                            stack_size = joint_config_stack.shape[0]
			    print(stack_size)
                            total_sample_size = total_sample_size + stack_size
                            for idx in range(stack_size):
                                new_row = list(joint_config_stack[idx, :])+\
                                                list(cube_poses_quat.flatten())
                                writer.writerow(new_row)
                        else:
                            print('No Valide Joint Configurations')
	    print(stack_size)
            print ("Sample Size is %i" % total_sample_size)
            print ("Config saved to file: " + self.csv_path)

    def get_random_cube_pose (self, p_invsisble=0.3):
        valid_poses = False
        while not valid_poses:
            x_buff = np.around(np.random.uniform(-0.5, 0.5, (3, 1)), decimals=3)
            # 0.7 is the table centre to world offset
            y_buff = np.around(np.random.uniform(-0.15, 0.25, (3, 1)), decimals=3)+0.59
            xy_buff = np.hstack((x_buff, y_buff))
            d1 = np.linalg.norm(xy_buff[0, :]-xy_buff[1, :])
            d2 = np.linalg.norm(xy_buff[0, :]-xy_buff[2, :])
            d3 = np.linalg.norm(xy_buff[1, :]-xy_buff[2, :])
            if d1>1.5*self.cube_length and d2>1.5*self.cube_length and d3>1.5*self.cube_length:
                valid_poses = True
        # table2world_xyz_offset = np.array([0, 0.7, 0.7125])
        cube_poses_quat = np.zeros((3,7))
        for i in range(0, 3):
            cube_poses_quat[i, 0] = xy_buff[i, 0]
            cube_poses_quat[i, 1] = xy_buff[i, 1]
            prob_token = random.uniform(0, 1)
            if prob_token < p_invsisble:
                z = self.table_height - 0.2
            else:
                z = self.cube2table_offset + self.table_height # z
            cube_poses_quat[i, 2] = z
            yaw = round(random.uniform(pi/2, 3*pi/2), 5)
            quaternion = tf.transformations.quaternion_from_euler(pi, 0, yaw)
            quaternion = np.around(quaternion, decimals=5)
            cube_poses_quat[i, 3:7] = quaternion
        return cube_poses_quat

    def get_joint_config_stack(self, h2t_6D_poses, X_T):
        joint_config_stack = None
        sample_size = h2t_6D_poses.shape[0]
        for i in range(sample_size):
            pose6d = h2t_6D_poses[i, :]
            T_X_H = self.pose6d2mat(pose6d)
            X_H = X_T.dot(T_X_H)
            # validate generated joint_config
            joint_config = self.ur5_kin.cmpt_elbow_up_ik(X_H)
            if joint_config is not None:
                config_in_range = np.prod((joint_config<self.joint_upper_limits)
                                    *(joint_config>self.joint_lower_limits))
                ee_pose_fk = self.ur5_kin.get_ee_pose(joint_config)
                error = np.linalg.norm(X_H-ee_pose_fk)
                if config_in_range and error<0.1:
                    joint_config = joint_config.reshape((1,6))
                    if joint_config_stack is None:
                        joint_config_stack = joint_config
                    else:
                        joint_config_stack = np.vstack((joint_config_stack,
                                                            joint_config))
        return joint_config_stack

    def pose6d2mat(self, pose6d):
        row = pose6d[3]
        pitch = pose6d[4]
        yaw = pose6d[5]
        mat = tf.transformations.euler_matrix(row, pitch, yaw)
        mat[0:3,3] = pose6d[0:3]
        return mat

    def quat2mat(self, quat):
        mat = tf.transformations.quaternion_matrix(quat[3:])
        mat[0:3,3] = quat[0:3]
        # print (mat)
        return mat

    def get_random_radius_vec(self, mean, std, low, high, size):
        norm2uni_ratio = 0.6
        norm_size = int(size*norm2uni_ratio)
        uniform_size = size - norm_size
        norm_samples = self.trunc_norm(mean, std, low, high, norm_size)
        uniform_samples = np.random.uniform(std, high, uniform_size)
        uniform_samples = uniform_samples.reshape((uniform_size,1))
        radius_vec = np.vstack((norm_samples, uniform_samples))
        radius_vec = np.around(radius_vec, decimals=4)
        return radius_vec

    def get_h2t_6D_poses(self, radius_vec, dense_sample_size):
        poses_dense = self.get_h2t_6D_poses_dense(dense_sample_size)
        poses_sparse = self.get_h2t_6D_poses_sparse(radius_vec)
        poses = np.vstack((poses_dense, poses_sparse))
        poses = np.around(poses, decimals=4)
        return poses

    def get_h2t_6D_poses_sparse(self, radius_vec):
        xyz_samples = None
        for i, radius in enumerate(radius_vec):
            if self.dataset_focus == 'global':
                chord_length = radius/3.0
            elif self.dataset_focus == 'local':
                chord_length = radius/5.0
            else:
                sys.exit("The Dataset Focus Needs to Be Either \'global\' or \'local\'")
            if i == 0:
                xyz_samples = self.sample_on_semi_sphere(radius, chord_length)
            else:
                new_xyz_mat = self.sample_on_semi_sphere(radius, chord_length)
                xyz_samples = np.vstack((xyz_samples, new_xyz_mat))
        sample_size = xyz_samples.shape[0]
        row = self.trunc_norm(0, 0.2, -pi/4, pi/4, sample_size) # r
        pitch = self.trunc_norm(0, 0.2, -pi/4, pi/4, sample_size)  # p
        yaw = self.trunc_norm(0, 0.57, -pi, pi, sample_size) # y
        sparse_poses = np.hstack((xyz_samples, row, pitch, yaw))
        return sparse_poses

    def get_h2t_6D_poses_dense(self, sample_size):
        x = np.random.uniform(-0.01, 0.01, (sample_size, 1))
        y = np.random.uniform(-0.04, 0.04, (sample_size, 1))
        z = np.random.uniform( -0.12, -0.04, (sample_size, 1))
        row = self.trunc_norm(0, 0.05, -0.2, 0.2, sample_size) # r
        pitch = self.trunc_norm(0, 0.05, -0.2, 0.2, sample_size)  # p
        yaw = self.trunc_norm(0, 0.13, -0.3, 0.3, sample_size) # y
        dense_poses = np.hstack((x, y, z, row, pitch, yaw))
        return dense_poses

    def sample_on_semi_sphere(self, radius, chord_length):
        delta = 2*asin(chord_length/(2*radius))
        phy =  np.arange(pi/2, pi, delta)
        phy = phy.reshape((phy.size, 1))
        theta = np.arange(0, 2*pi, delta)
        theta = theta.reshape((theta.size, 1))
        x_mat = radius*np.sin(phy).dot(np.cos(np.transpose(theta)))
        y_mat = radius*np.sin(phy).dot(np.sin(np.transpose(theta)))
        z_mat = radius*np.cos(phy).dot(np.ones((1, theta.size)))
        x, y, z = x_mat.flatten(), y_mat.flatten(), z_mat.flatten()
        x, y, z = x.reshape((x.size, 1)), y.reshape((y.size, 1)),\
                                            z.reshape((z.size, 1))
        xyz_mat = np.hstack((x, y, z))
        return xyz_mat

    def trunc_norm(self, mean, std, low, high, size):
        rand_number = None
        rand_col = np.zeros((size, 1))
        sample_idx = 0
        while sample_idx < size:
            rand_number = np.random.normal(mean, std)
            if rand_number >=low and rand_number<=high:
                rand_col[sample_idx, 0] = rand_number
                sample_idx += 1
        rand_col = np.around(rand_col, decimals=4)
        return rand_col

    def check_file_path(self):
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        elif os.path.isfile(self.csv_path):
            sys.exit("File Already Exists, Exiting...")

    def load_metadata(self):
        metadata_path = os.path.expanduser('~/apclab_dev-RAL_Cleanup/src/config.yml')
        with open(metadata_path, 'r') as config_file:
            try:
                config_metadata = yaml.load(config_file)
            except yaml.YAMLError as exc:
                print(exc)
        return config_metadata


def main(args):
    print(args)
    cube_sample_size = 3 #int(args[1])
    file_name = 'b2.csv' #args[2]
    dataset_focus = 'global' #args[3]
    collector = ConfigGenerator(cube_sample_size, file_name, dataset_focus)
    collector.generate_scene_config()
    if dataset_focus == 'local':
        print('Searching Low Lyap Scene Configs...')
        dataset = LowLyapSearch(file_name)
        dataset.get_imdb()


if __name__ == '__main__':
    main(sys.argv)
