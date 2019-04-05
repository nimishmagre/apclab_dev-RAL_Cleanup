#!/usr/bin/env python
import h5py
import os
import cv2
import sys
import numpy as np
import rospkg
from shutil import copyfile
from tqdm import tqdm
import pandas as pd
import os
from skimage import io
import cv2
rospack = rospkg.RosPack()
kin_path = rospack.get_path('ur_description')
sys.path.append(os.path.join(kin_path, 'ur5_kin'))
from ur5_kin_scaled import UR5Kin
from zoom_at_hand import ZoomAtHand

class CreateH5Dataset():

    def __init__(self, folder_name, dataset_focus):
        """
        Args:
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        dir_path = os.path.expanduser('~/apcNet_dataset/dataset')
        self.root_dir = os.path.join(dir_path, folder_name)
        self.img_zoomer = ZoomAtHand()
        self.csv_file = 'imdb_one_shot.csv' 
        self.file_path = os.path.join(self.root_dir, self.csv_file)
        self.imdb = pd.read_csv(self.file_path, skip_blank_lines=True)
        self.ur5_kin = UR5Kin(t_scale=6)
        self.n_samples = self.imdb.shape[0]
        self.d_theta = 0.05
        hf_path = os.path.join(dir_path, folder_name+'_imdb'+'.hdf5')
        self.hf = h5py.File(hf_path, 'w')
        self.dataset_focus = dataset_focus

    def init_datasets(self):
        self.hf.create_dataset('left_image',
                            shape = (self.n_samples, 224, 224, 3),
                            dtype=np.dtype('uint8'))
        self.hf.create_dataset('right_image',
                            shape = (self.n_samples, 224, 224, 3),
                            dtype=np.dtype('uint8'))
        self.hf.create_dataset('vp7',
                            shape = (self.n_samples, 7, 72),
                            dtype=np.dtype('float32'))
        self.hf.create_dataset('pd_lyap',
                            shape = (self.n_samples, 6),
                            dtype=np.dtype('float32'))
        self.hf.create_dataset('lyap',
                                shape = (self.n_samples, ),
                                dtype=np.dtype('float32'))
        self.hf.create_dataset('vel',
                                shape = (self.n_samples, 6),
                                dtype=np.dtype('float32'))
        self.hf.create_dataset('joint_config',
                                shape = (self.n_samples, 6),
                                dtype=np.dtype('float32'))

    def get_vp7(self, joint_config):
        vp1 = self.ur5_kin.get_veced_joint_poses(joint_config)
        # compute joint 6D v2
        vp_batch = np.zeros((7, 72))
        vp_batch[0, :] = vp1
        for i in range(6):
            d_joint_vec = np.zeros((6, 1))
            d_joint_vec[i, 0] = self.d_theta
            vp_batch[i + 1, :] = self.ur5_kin.get_veced_joint_poses(joint_config + d_joint_vec)
        return vp_batch

    def img_preprocessing(self, img_path):
        cv2_img = cv2.imread(img_path) #BRG, HxWxC
        cv2_img = self.get_centre_square_crop(cv2_img)
        cv2_img = cv2.resize(cv2_img, (224, 224), interpolation=cv2.INTER_CUBIC)
        # cv2.imshow('test_img', cv2_img)
        # cv2.waitKey()
        cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)
        return cv2_img

    def get_centre_square_crop(self, cv2_img):
        H, W, C = cv2_img.shape
        cropsize = H//2
        cropped_img = cv2_img[:, W//2-cropsize:W//2+cropsize,:]
        return cropped_img

    def get_zoomed_image(self, img_path, joint_config):
        cv2_img = cv2.imread(img_path) #BRG, HxWxC
        zoomed_img = self.img_zoomer.get_zoomed_image(cv2_img, joint_config, offset = 65)
        zoomed_img = cv2.resize(zoomed_img, (224, 224), interpolation=cv2.INTER_CUBIC)
        # cv2.imshow('test_img', zoomed_img)
        # cv2.waitKey()
        zoomed_img = cv2.cvtColor(zoomed_img, cv2.COLOR_BGR2RGB)
        return zoomed_img

    def cmpt_lyap_vctrl(self, lyap, J):
        J = J.reshape((1,6))
        J_pinv = np.linalg.pinv(J)
        lyap_vctrl = J_pinv*lyap
        lyap_vctrl = lyap_vctrl.reshape((6))
        return lyap_vctrl

    def convert2dataset(self):
        self.init_datasets()
        for i in tqdm(range(0, self.n_samples)):
            joint_config = self.imdb.iloc[i, 9:].values  #self.imdb.iloc[i, 2:8].values
            joint_config = joint_config.reshape((6,1))
            right_img_path = os.path.join(self.root_dir, self.imdb.iloc[i, 0])
            leftt_img_path = os.path.join(self.root_dir, self.imdb.iloc[i, 1])
            if self.dataset_focus == 'global':
                right_img = self.img_preprocessing(right_img_path)
                left_img = self.img_preprocessing(right_img_path)
            elif self.dataset_focus == 'local':
                right_img = self.get_zoomed_image(right_img_path, joint_config)
		left_img = self.get_zoomed_image(left_img_path, joint_config)
            else:
                sys.exit("The Dataset Focus Needs to Be Either \'global\' or \'local\'")
            self.hf['right_image'][i, ...] = right_img
            self.hf['left_image'][i, ...] = left_img
            self.hf['vp7'][i, ...] = self.get_vp7(joint_config)
            J = self.imdb.iloc[i, 3:9].values.astype('float32') #self.imdb.iloc[i, 3:9].values.astype('float32')
            lyap = self.imdb.iloc[i, 3].astype('float32') #self.imdb.iloc[i, 3].astype('float32')
            self.hf['vel'][i, ...] = self.cmpt_lyap_vctrl(lyap, J)
        self.hf['joint_config'][...] = self.imdb.iloc[:, 9:].values.astype('float32') #self.imdb.iloc[:, 2:8].values.astype('float32')
        self.hf['pd_lyap'][...] = self.imdb.iloc[:, 3:9].values.astype('float32')
        self.hf['lyap'][...] = self.imdb.iloc[:, 2].values.astype('float32')
        self.hf.close()

if __name__=='__main__':
    folder_name = 'dset' #sys.argv[1]
    dataset_focus = 'global' #sys.argv[2]
    hdf5converter = CreateH5Dataset(folder_name, dataset_focus)
    hdf5converter.convert2dataset()
