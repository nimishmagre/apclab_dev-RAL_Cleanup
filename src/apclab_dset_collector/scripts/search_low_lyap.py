#!/usr/bin/env python
import sys
import os
import rospkg
rospack = rospkg.RosPack()
lyap_path = rospack.get_path('apclab_control')
kin_path = rospack.get_path('ur_description')
sys.path.append(os.path.join(lyap_path, 'scripts/lyap_funcs'))
sys.path.append(os.path.join(kin_path, 'ur5_kin'))
import tf
import csv
import os
import yaml
from tqdm import tqdm
import numpy as np
from ur5_kin_scaled import UR5Kin
from lyap_eh import LyapEH
# Notation: X represents homogeneous transformation, A_X_B represents coordinate
# mapping of frame B w.r.t A, A can be omitted if A is the world fixed frame.
# rigid-transformation is represented with triple index notation: AC_X_B. it
# represents the motion from frame C to B in A.

# This get the homogeneous transformation matrix from quaternion and translation vector
# Q: list [x,y,z,w], t:[x,y,z]

class LowLyapSearch():
    """
    This Class Interprets raw data collected from the simulator to the CNN imdb.

    A function provided by this class is augmenting the dataset with extra six
    samples for each sample: each additional sample has identical image as its
    source, yet one of its joint angles theta_i = theta_i + delta. Joint poses
    and Lyapunov function is re-computed based on the new joint config.

    The intuition behind this is if we can predict Lyap up to a certain
    precision, we can approximate the partial derivative and thus the velocity
    controller of the robot.
    """
    def __init__(self, config_file_name):
        # load_metadata
        config_metadata = self.load_metadata()
        self.table_height = float(config_metadata['table_height'])
        self.lyap_threshold = float(config_metadata['low_lyap_threshold'])
        self.grasp_offset = float(config_metadata['grasp_offset'])
        self.t_scale = float(config_metadata['t_scale'])

        self.config_file_name = config_file_name
        self.data_path = os.path.expanduser('~/apcNet_dataset/scene_config/')
        self.output_path = self.data_path
        self.cube_pose_header = ["cube1_x", "cube1_y","cube1_z", "cube1_a",
                                "cube1_b", "cube1_c", "cube1_d",
                                "cube2_x", "cube2_y","cube2_z", "cube2_a",
                                "cube2_b", "cube2_c", "cube2_d",
                                "cube3_x", "cube3_y","cube3_z", "cube3_a",
                                "cube3_b", "cube3_c", "cube3_d",]
        self.joint_names = ['s_pan','s_lift','elbow','w1','w2','w3']
        # self.check_file_path(self.output_path)

    def get_XT_from_Qt(self, Q, t):
        R = tf.transformations.quaternion_matrix(Q)
        I = tf.transformations.identity_matrix() # 4x4 Identity_matrix
        X = R+tf.transformations.translation_matrix(t)-I
        X[2, 3] = X[2, 3]+self.grasp_offset
        X = self.scale_translation(X)
        return X

    def get_XTs_from_row(self, row):
        Q1 = [row['cube1_a'], row['cube1_b'], row['cube1_c'], row['cube1_d']]
        t1 = [row['cube1_x'], row['cube1_y'], row['cube1_z']]
        Q2 = [row['cube2_a'], row['cube2_b'], row['cube2_c'], row['cube2_d']]
        t2 = [row['cube2_x'], row['cube2_y'], row['cube2_z']]
        Q3 = [row['cube3_a'], row['cube3_b'], row['cube3_c'], row['cube3_d']]
        t3 = [row['cube3_x'], row['cube3_y'], row['cube3_z']]
        XT1 = self.get_XT_from_Qt(Q1, t1)
        XT2 = self.get_XT_from_Qt(Q2, t2)
        XT3 = self.get_XT_from_Qt(Q3, t3)
        X_T_candidates = [XT1, XT2, XT3]
        X_Ts = []
        for X_T_temp in X_T_candidates:
            if X_T_temp[2,3]>self.table_height*self.t_scale:
                X_Ts.append(X_T_temp)
        return X_Ts

    def get_joint_config_from_row(self, row):
        joint_config = [row['s_pan'],
	                   row['s_lift'],
                       row['elbow'],
                       row['w1'],
                       row['w2'],
                       row['w3']]
        joint_config = np.array([float(i) for i in joint_config])
        joint_config = np.around(joint_config, decimals=4)
        joint_config = joint_config.reshape((6,1))
        return joint_config

    def get_imdb(self):
        ur5_kin = UR5Kin(self.t_scale)
        raw_data_file = os.path.join(self.data_path, self.config_file_name)
        output_file_name = self.config_file_name[:-4]+'_lowlyap.csv'
        output_file_path = os.path.join(self.output_path, output_file_name)
        low_lyap_count = 0
        try:
            os.remove(output_file_path)
        except OSError:
            pass
        with open(raw_data_file) as raw_data, open(output_file_path, "a") as output_file:
            reader = list(csv.DictReader(raw_data))
            writer = csv.writer(output_file)
            output_hearder =  self.joint_names + self.cube_pose_header
            writer.writerow(output_hearder)
            for row in tqdm(reader):
                joint_config = self.get_joint_config_from_row(row)
                X_H = ur5_kin.get_ee_pose(joint_config)
                X_Ts = self.get_XTs_from_row(row)
                lyap_func = LyapEH()
                lyap_buff = np.array([])
                for X_T in X_Ts:
                    if X_T[2,3]>self.table_height:
                        lyap_func.X_T = X_T
                        lyap_temp = lyap_func.cmpt_Lyap(X_H)
                        lyap_buff = np.append(lyap_buff, lyap_temp)
                lyap = np.amin(lyap_buff)
                lyap_func.X_T = X_Ts[np.argmin(lyap_buff)]
                # dataset augmentation: if True, the generated imdb will be 6x bigger
                if lyap < self.lyap_threshold:
                    writer.writerow(row.values())
                    low_lyap_count += 1
            print('Low Lyap Sample Size: %i' % low_lyap_count)

    def scale_translation(self, X):
        X[0:3, 3] = self.t_scale*X[0:3, 3]
        return X

    def check_file_path(self, dir_path):
        if os.path.exists(dir_path):
            print('folder already exists')
        else:
            os.makedirs(dir_path)

    def load_metadata(self):
        metadata_path = os.path.expanduser('~/apclab_dev-RAL_Cleanup/src/config.yml')
        with open(metadata_path, 'r') as config_file:
            try:
                config_metadata = yaml.load(config_file)
            except yaml.YAMLError as exc:
                print(exc)
        return config_metadata

def main(args):
    config_file_name = args[1]
    dataset = LowLyapSearch(config_file_name)
    dataset.get_imdb()

if __name__ == "__main__":
    main(sys.argv)
