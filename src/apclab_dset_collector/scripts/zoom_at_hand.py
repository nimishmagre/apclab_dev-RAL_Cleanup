#!/usr/bin/env python
import sys
import os
import rospkg
rospack = rospkg.RosPack()
kin_path = rospack.get_path('ur_description')
sys.path.append(os.path.join(kin_path, 'ur5_kin'))
import tf
import csv
import os
from tqdm import tqdm
import numpy as np
from ur5_kin_scaled import UR5Kin
import cv2
import pdb
import yaml
# Notation: X represents homogeneous transformation, A_X_B represents coordinate
# mapping of frame B w.r.t A, A can be omitted if A is the world fixed frame.
# rigid-transformation is represented with triple index notation: AC_X_B. it
# represents the motion from frame C to B in A.

# This get the homogeneous transformation matrix from quaternion and translation vector
# Q: list [x,y,z,w], t:[x,y,z]

class ZoomAtHand():
    """
    This Class Zoom in at proximity of hand
    """
    def __init__(self):
        config_metadata = self.load_metadata()
        # load camera intrinsics
        self.cam_f = float(config_metadata['cam_f'])
        self.cam_res = map(float, config_metadata['cam_res'])
        # load camera extrinsics
        self.cam_trans = map(float, config_metadata['cam_trans'])
        self.cam_quat = map(float, config_metadata['cam_quat'])

    def get_cam_porj_matrix(self):
        f = self.cam_f
        x0 = self.cam_res[0]/2.0
        y0 = self.cam_res[1]/2.0
        C = np.array([[f, 0.0, x0],
                    [0.0, f, y0],
                    [0.0, 0.0, 0.0]])
        w_X_c = self.get_X_from_Qt(self.cam_quat, self.cam_trans)
        c_X_w = np.linalg.inv(w_X_c)
        R_t = c_X_w[0:3, :]
        return C, R_t

    def gazebo_cam_coor_conversion(self, p):
        R0 = np.array([[0.0, -1.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [1.0, 0.0, 0.0]])
        p = R0.dot(p)
        return p

    def get_zoomed_image(self, cv2_img, joint_config, offset=65):
        ur5_kin = UR5Kin()
        C, R_t = self.get_cam_porj_matrix()
        X_H = ur5_kin.get_ee_pose(joint_config)
        # get ee [X, Y, Z, 1]^T
        hand_XYZ1 = X_H[:,3]
        # get camera matrix
        c_p_gazebo = R_t.dot(hand_XYZ1)
        c_p_cv = self.gazebo_cam_coor_conversion(c_p_gazebo)
        c_p_cv = c_p_cv/c_p_cv[-1]
        img_coor_uv1 = (C.dot(c_p_cv)).astype(int)
        # print(img_coor_uv1)
        u = img_coor_uv1[0]
        v = img_coor_uv1[1]
        # print(u,v)
        # read image
        # crop image
        # cv2.rectangle(cv2_img,(u,v),(u+1,v+1),(0,255,0),3)
        # cv2.rectangle(cv2_img,(u-offset,v-offset+10),(u+offset,v+offset+10),(0,0,255),2)t
        # cv2.imshow('test', new_img)
        # cv2.waitKey(0)
        zoomed_img_bgr = cv2_img[v-offset+10:v+offset+10,
                            u-offset:u+offset,:]
        return zoomed_img_bgr

    def load_metadata(self):
        metadata_path = os.path.expanduser('~/apclab_dev-RAL_Cleanup/src/config.yml')
        with open(metadata_path, 'r') as config_file:
            try:
                config_metadata = yaml.load(config_file)
            except yaml.YAMLError as exc:
                print(exc)
        return config_metadata

    #############
    # Tool Box  #
    #############
    def get_X_from_Qt(self, Q, t):
        R = tf.transformations.quaternion_matrix(Q)
        I = tf.transformations.identity_matrix() # 4x4 Identity_matrix
        X = R+tf.transformations.translation_matrix(t)-I
        return X

# def main(args):
#     folder_name = args[1]
#     dataset_dir = os.path.expanduser('~/apcNet_dataset/dataset/')
#     data_path = os.path.join(dataset_dir, folder_name)
#     dataset = ZoomAtHand(data_path, data_path)
#
# if __name__ == "__main__":
#     main(sys.argv)
