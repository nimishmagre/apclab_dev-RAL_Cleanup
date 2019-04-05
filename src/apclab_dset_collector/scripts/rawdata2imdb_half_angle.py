#!/usr/bin/env python
import sys
import os
import yaml
import numpy as np
import csv
import cv2
import math
from tqdm import tqdm
import rospkg
rospack = rospkg.RosPack()
lyap_path = rospack.get_path('apclab_control')
kin_path = rospack.get_path('ur_description')
sys.path.append(os.path.join(lyap_path, 'scripts/lyap_funcs'))
sys.path.append(os.path.join(kin_path, 'ur5_kin'))
import tf
from ur5_kin_scaled import UR5Kin
from lyap_eh import LyapEH

# Notation: X represents homogeneous transformation, A_X_B represents coordinate
# mapping of frame B w.r.t A, A can be omitted if A is the world fixed frame.
# rigid-transformation is represented with triple index notation: AC_X_B. it
# represents the motion from frame C to B in A.

# This get the homogeneous transformation matrix from quaternion and translation vector
# Q: list [x,y,z,w], t:[x,y,z]

class RawData2IMDB():
    """
    This Class Interprets raw data collected from the simulator to the CNN imdb.
    """
    def __init__(self, data_path, output_path, dataset_focus):
        self.data_path = data_path
        self.output_path = output_path
        self.output_needs_header = True
        self.dataset_focus = dataset_focus
        config_metadata = self.load_metadata()
        self.table_height = float(config_metadata['table_height'])
        self.low_lyap_threshold = float(config_metadata['low_lyap_threshold'])
        self.grasp_offset = float(config_metadata['grasp_offset'])
        self.t_scale = float(config_metadata['t_scale'])
        self.ur5_kin = UR5Kin(self.t_scale)
        self.lyap_func = LyapEH()


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
        joint_config = [row['shoulder_pan_joint'],
        	                   row['shoulder_lift_joint'],
                               row['elbow_joint'],
                               row['wrist_1_joint'],
                               row['wrist_2_joint'],
                               row['wrist_3_joint']]
        joint_config = np.array([float(i) for i in joint_config])
        joint_config = np.around(joint_config, decimals=4)
        joint_config = joint_config.reshape((6,1))
        return joint_config

    def get_imdb(self):
        raw_data_file = os.path.join(self.data_path, 'armbot_dataset_3cubes.csv')
        output_file_name = 'imdb_one_shot.csv'
        output_file_path = os.path.join(self.output_path, output_file_name)
        try:
            os.remove(output_file_path)
        except OSError:
            pass
        with open(raw_data_file) as raw_data, open(output_file_path, "a") as output_file:
            reader = list(csv.DictReader(raw_data))
            writer = csv.writer(output_file)
            for row in reader[10:11]:
                image_name = row['image_name']
                joint_config = self.get_joint_config_from_row(row)
                X_H = self.ur5_kin.get_ee_pose(joint_config)
                X_Ts = self.get_XTs_from_row(row)
                # veced_poses = ur5_kin.get_veced_joint_poses(joint_config)
                lyap_buff = np.array([])
                for X_T in X_Ts:
                    if X_T[2,3]>self.table_height:
                        self.lyap_func.X_T = X_T
                        lyap_temp = self.lyap_func.cmpt_Lyap(X_H)
                        lyap_buff = np.append(lyap_buff, lyap_temp)
                lyap = np.amin(lyap_buff)
                self.lyap_func.X_T = X_Ts[np.argmin(lyap_buff)]
                # dataset augmentation: if True, the generated imdb will be 6x bigger
                output_hearder = ['image_name', 'lyap', 'pd_1', 'pd_2','pd_3',
                                    'pd_4', 'pd_5', 'pd_6', 's_pan', 's_lift',
                                     'elbow', 'w1', 'w2','w3']
                if self.output_needs_header:
                    writer.writerow(output_hearder)
                    self.output_needs_header = False
                if self.dataset_focus == 'global':
                    meet_collect_criterion = lyap >= 5e-4
                    new_image_name  = image_name
                elif self.dataset_focus == 'local':
                    meet_collect_criterion = lyap <= self.low_lyap_threshold
                else:
                    sys.exit("The Dataset Focus Needs to Be Either \'global\' or \'local\'")
                if meet_collect_criterion:
                    pd_Lyap = list()
                    for i in range(6):
                        self.cmpt_mean_value_point(joint_config, i, self.lyap_func.X_T, 0.05)
                        pd_matrix = self.ur5_kin.get_pd_X_H(joint_config, i)
                        pd_analyt = self.lyap_func.cmpt_pd_Lyap(X_H, pd_matrix)
                        print('pd_analyt: %.4f' % pd_analyt)
                        pd_Lyap.append(pd_analyt)
                    new_row = [image_name]+[lyap]+pd_Lyap+[row['shoulder_pan_joint'],
                    	                   row['shoulder_lift_joint'],
                                           row['elbow_joint'],
                                           row['wrist_1_joint'],
                                           row['wrist_2_joint'],
                                           row['wrist_3_joint']]
                    writer.writerow(new_row)

    def scale_translation(self, X):
        X[0:3, 3] = self.t_scale*X[0:3, 3]
        return X

    def cmpt_mean_value_point(self, joint_config, joint_index, X_T, delta):
        # LHS
        G = np.copy(X_T)
        A, B = self.ur5_kin.get_constant_DH_mats(joint_config, joint_index)
        inv_A = np.linalg.inv(A)
        inv_B = np.linalg.inv(B)
        # Rotation matrix Z Linearisation
        C = np.zeros([4, 4]); C[0, 0] = -1; C[1, 1] = -1
        D = np.zeros([4, 4]); D[0, 1] = 1; D[1, 0] = -1
        E = np.zeros([4, 4]); E[0, 1] = -1; E[1, 0] = 1;
        F = np.zeros([4, 4]); F[0, 1] = 1; F[1, 0] = -1
        H = np.zeros([4, 4]); H[0, 0] = 1; H[1, 1] = 1
        # print(C, D, E, F, H)
        # for a quadratic equation ax^2+x^2+c
        M1 = np.transpose(inv_B.dot(inv_A).dot(G))
        M2 = np.transpose(inv_B.dot(C).dot(inv_A).dot(G))
        M3 = np.transpose(inv_B.dot(D).dot(inv_A).dot(G))
        M4 = inv_B.dot(E).dot(inv_A).dot(G)
        M5 = inv_B.dot(F).dot(inv_A).dot(G)
        M6 = inv_B.dot(H).dot(inv_A).dot(G)
        a = np.trace(M2.dot(M5))
        b = np.trace(M3.dot(M5)+M2.dot(M6))
        c = np.trace(M1.dot(M5)+M2.dot(M4)-M5)
        d = np.trace(M3.dot(M4)+M1.dot(M6)-M6)
        e = np.trace(M1.dot(M4)-M4)
        # RHS
        perturbed_joint_config = np.copy(joint_config)
        perturbed_joint_config[joint_index, 0] = perturbed_joint_config[joint_index, 0]+delta
        lyap_left = self.lyap_func.cmpt_Lyap(self.ur5_kin.get_ee_pose(joint_config))
        lyap_right = self.lyap_func.cmpt_Lyap(self.ur5_kin.get_ee_pose(
                                                            perturbed_joint_config))
        slope = (lyap_right-lyap_left)/(delta)
        e = e+slope/2.0
        # coefficients p[] for quartic equation
        p = [4.0*a+2.0*c+e, 4.0*b+2.0*d, 2.0*c+2.0*e, 2.0*d, e]
        roots = np.roots(p)
        real_roots = roots[np.isreal(roots)].real
        mean_value_points = 2.0*np.arctan(real_roots)
        left_end = joint_config[joint_index, 0] - 5*delta
        right_end = joint_config[joint_index, 0] + 5*delta
        mean_value_point = mean_value_points[(left_end<mean_value_points) &
                                                    (mean_value_points<right_end)]
        print(mean_value_point)
        print(joint_config[joint_index,0])
        c  = np.copy(joint_config)
        c[joint_index, 0] = mean_value_point[0]
        pd_matrix = self.ur5_kin.get_pd_X_H(c, joint_index)
        pd_analyt = self.lyap_func.cmpt_pd_Lyap(self.ur5_kin.get_ee_pose(c), pd_matrix)
        print('slope: %.4f, mvp_pd: %.4f' % (slope, pd_analyt))
        # return mean_value_point

    def load_metadata(self):
        metadata_path = os.path.expanduser('~/apclab_dev/src/config.yml')
        with open(metadata_path, 'r') as config_file:
            try:
                config_metadata = yaml.load(config_file)
            except yaml.YAMLError as exc:
                print(exc)
        return config_metadata

def main(args):
    folder_name = args[1]
    dataset_focus = args[2]
    dataset_dir = os.path.expanduser('~/apcNet_dataset/dataset/')
    data_path = os.path.join(dataset_dir, folder_name)
    dataset = RawData2IMDB(data_path, data_path, dataset_focus)
    dataset.get_imdb()

if __name__ == "__main__":
    main(sys.argv)
