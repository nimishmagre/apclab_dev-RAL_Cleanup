#!/usr/bin/env python

### Sys Pkg
import sys
import rospkg
import os
import time
import math
import numpy as np
### Pytorch Pkg
import torch
import yaml
from torchvision import transforms
### APCNet Pkg
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
sys.path.append(os.path.join(rospack.get_path('apclab_dset_collector'), 'scripts'))
sys.path.append(os.path.join(rospack.get_path('apclab_control'), 'scripts/lyap_funcs'))
sys.path.append(os.path.expanduser('~/apcNet_dev/apcNet_Naive_E2E'))
from pose_msg_converter import *
from ur5_kin_scaled import UR5Kin
from lyap_eh import LyapEH
from apcNet_naive_e2e import NaiveE2E
### ROS Sys Pkg
import rospy
import cv2
import message_filters
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetLinkState

bridge = CvBridge()

class APCNetHowSwapSim():

    def __init__(self, cnn_infer):
        # load metadata
        config_metadata = self.load_metadata()
        self.t_scale = float(config_metadata['t_scale'])
        self.grasp_offset = float(config_metadata['grasp_offset'])
        self.table_height = float(config_metadata['table_height'])
        # Load Local and Global Net
        if cnn_infer == "infer":
            self.apcNet = NaiveE2E()
            self.apcNet.model_fld = 'naive_e2e_test02'
            self.apcNet.load_ckpt(load_best=True)
        self.ur5_kin = UR5Kin(self.t_scale)
        self.lyap_func = LyapEH()
        self.apcNet_infer = cnn_infer
        self.image_topic = "/sim_camera/image_raw"
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Init msg for publishing
        self.joint_traj = JointTrajectory()
        self.joint_traj.joint_names = self.joint_names
        self.lyap_vctrl_old = np.zeros([6,1])
        # Approximated Synchronised Topics
        self.image_capturer_sub = message_filters.Subscriber(
            self.image_topic, Image)
        self.q_sub = message_filters.Subscriber(
            "/arm_controller/state", JointTrajectoryControllerState)
        self.synced_msgs = message_filters.ApproximateTimeSynchronizer(
                        [ self.image_capturer_sub, self.q_sub], 10, 0.005)
        self.synced_msgs.registerCallback(self.image_callback)
        self.sim_joint_command_pub = rospy.Publisher("/arm_controller/command",
                                                    JointTrajectory, queue_size=10)
        self.real_joint_command_pub = rospy.Publisher("/ur_driver/joint_speed",
                                                    JointTrajectory, queue_size=10)

    def image_callback(self, image_msg, joint_msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(image_msg, "rgb8")
        except CvBridgeError, e:
            print(e)
        else:
            start_time = time.time()
            joint_config = self.joint_msg2q(joint_msg)
            vel_command = self.cmpt_vel_command(cv2_img, joint_config)
            self.joint_traj.points = [JointTrajectoryPoint(positions = joint_config,
                                        velocities= -vel_command,
                                        time_from_start=rospy.Duration(0.1))]
            infer_time = time.time() - start_time
            refresh_rate = 1.0/infer_time
            print('Refresh Rate %4.fHz' % refresh_rate)

    def cmpt_vel_command(self, cv2_img, joint_config):
            X_H = self.ur5_kin.get_ee_pose(joint_config) # scaled
            X_T = self.get_best_target_preproc(X_H) # scaled
            self.lyap_func.X_T = X_T
            if self.apcNet_infer == 'infer':
                img = self.cv2_img2torch(cv2_img)
                lyap, pd = self.infer_J(img, joint_config, self.apcNet)
                # J_GT = self.cmpt_analyt_J(joint_config, X_T, X_H)
                lyap_GT = self.lyap_func.cmpt_Lyap(X_H)
                print ("Lyap Inferred: %.5f" % lyap)
                print ("Lyap GT: %.5f" % lyap_GT)
                lyap_vctrl_temp = self.cmpt_lyap_vctrl(lyap, pd)
            elif self.apcNet_infer == 'analyt':
                J = self.cmpt_analyt_J(joint_config, X_T, X_H)
                lyap = self.lyap_func.cmpt_Lyap(X_H)
                lyap_vctrl_temp = self.cmpt_lyap_vctrl(lyap, J)
            else:
                print ("####### Warning !!! #######")
                print('   Mode not supported!!! \nChoose \"infer\" or \"analyt\" ')
                print ("###########################")
                rospy.signal_shutdown('Quit')
            momentum = 0.1
            lyap_vctrl = momentum*self.lyap_vctrl_old+(1-momentum)*lyap_vctrl_temp
            self.lyap_vctrl_old = lyap_vctrl
            ############# Schedule Gain ###############
            if lyap > 0.05:
                phy = 1
            else:
                phy = 0
            vel_command = -phy*lyap_vctrl
            print ("Vel Controller:")
            print (vel_command)
            print ('Joint_Config')
            print (joint_config)
            print ('\n')
            # print[round(lyap_global, 5)] + [round(lyap_local, 5)] + map(float, list(joint_config.reshape(6)))
            return vel_command

    def scale_translation(self, X):
        X[0:3, 3] = self.t_scale*X[0:3, 3]
        return X

    def get_best_target_preproc(self, X_H):
        X_T1 = self.get_preproc_target_pose('cube1_link', 'world')
        X_T2 = self.get_preproc_target_pose('cube2_link', 'world')
        X_T3 = self.get_preproc_target_pose('cube3_link', 'world')
        X_T_candidates = [X_T1, X_T2, X_T3]
        X_Ts = []
        for X_T_temp in X_T_candidates:
            if X_T_temp[2,3]>self.table_height*self.t_scale:
                X_Ts.append(X_T_temp)
        lyap_buff = np.array([])
        for X_T in X_Ts:
            if X_T[2,3]>self.table_height:
                self.lyap_func.X_T = X_T
                lyap_temp = self.lyap_func.cmpt_Lyap(X_H)
                lyap_buff = np.append(lyap_buff, lyap_temp)
        X_T = X_Ts[np.argmin(lyap_buff)]
        return X_T

    def get_preproc_target_pose(self, link_name, reference_frame):
        X_T = self.get_link_pose(link_name, reference_frame)
        X_T[2,3] = X_T[2,3] + self.grasp_offset
        X_T = self.scale_translation(X_T)
        return X_T

    def infer_J(self, img, joint_config, apcNet):
        ur5_kin_scaled = UR5Kin(self.t_scale)
        with torch.no_grad():
            apcNet.net = apcNet.net.eval()
            joint_config = joint_config.reshape(6)
            joint_config = self.np2torch(joint_config).cuda()
            joint_config = joint_config.unsqueeze(0)
            lyap, pd = apcNet.net.infer(img.cuda(), joint_config)
        return lyap, pd

    def cmpt_lyap_vctrl(self, lyap, pd):
        if lyap > 0:
            phy = 0.5*min(1/lyap, 1)
            lyap_vctrl = phy*pd
        else:
            lyap_vctrl = 0.0*pd
        return lyap_vctrl.reshape((6,1))

    def cmpt_analyt_J(self, joint_config, X_T, X_H):
        self.lyap_func.X_T = X_T
        analyt_J = np.zeros((1,6))
        for i in range(6):
            pd_X_H = self.ur5_kin.get_pd_X_H(joint_config, i)
            analyt_J[0, i] = self.lyap_func.cmpt_pd_Lyap(X_H, pd_X_H)
        # print analyt_J
        return analyt_J

    ############
    # Tool Box #
    ############
    def joint_msg2q(self, joint_msg):
        joint_config = np.asarray(joint_msg.actual.positions)
        joint_config = np.around(joint_config, decimals=4)
        joint_config = joint_config.reshape((6,1))
        return joint_config

    def cv2_img2torch(self, cv_img):
        img = np.zeros(np.flipud(cv_img.shape))
        crop_size = cv_img.shape[0]

        preprocess = transforms.Compose([transforms.ToPILImage(),
                               transforms.CenterCrop(crop_size),
                               transforms.Resize((224, 224)),
                               transforms.ToTensor(),
                               transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                    std=[0.229, 0.224, 0.225])])
        img = preprocess(cv_img)
        img = img.unsqueeze(0)
        return img

    def np2torch(self, v):
        v = torch.from_numpy(v)
        v = v.type(torch.FloatTensor)
        return v

    def get_link_pose(self, link_name, reference_frame):
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            current_link_state = rospy.ServiceProxy(
                '/gazebo/get_link_state', GetLinkState)
            link_state = current_link_state(link_name, reference_frame)
            link_pose = link_state.link_state
            return pose_msg2se3(link_pose)
        except rospy.ServiceException, e:
            print "Service call failed: %s"

    def load_metadata(self):
        metadata_path = os.path.expanduser('~/apclab_dev/src/config.yml')
        with open(metadata_path, 'r') as config_file:
            try:
                config_metadata = yaml.load(config_file)
            except yaml.YAMLError as exc:
                print(exc)
        return config_metadata

########################################################
def main(args):
    rospy.init_node("apc_controller", anonymous=True)
    cnn_infer = args[1]
    vel_controller = APCNetHowSwapSim(cnn_infer)
    while not rospy.is_shutdown():
        try:
            vel_controller.sim_joint_command_pub.publish(vel_controller.joint_traj)
            rospy.sleep(0.008) # 125 Hz
        except KeyboardInterrupt:
            print "\'Ctrl+C\', Force Quit..."
            sys.exit()

if __name__ == "__main__":
    main(sys.argv)
