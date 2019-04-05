#!/usr/bin/env python

import os
# Sys Pkg
import sys

import numpy as np
import rospkg
import os
import time
import math
import numpy as np
import yaml
### Pytorch Pkg
import torch
from torchvision import transforms

# APCNet Pkg
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
sys.path.append(
    os.path.join(rospack.get_path('apclab_dset_collector'), 'scripts'))
sys.path.append(
    os.path.join(rospack.get_path('apclab_control'), 'scripts/lyap_funcs'))
sys.path.append(os.path.expanduser('~/apcNet_dev/apcNet_Direct_Siam'))
from pose_msg_converter import *
from ur5_kin_scaled import UR5Kin
from zoom_at_hand import ZoomAtHand
from apcNet_direct_siamese import DirectSiamese
# ROS Sys Pkg
import cv2
import rospy
import message_filters
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


class APCNetHowSwapReal:

    def __init__(self):
        # load metadata
        config_metadata = self.load_metadata()
        self.t_scale = float(config_metadata['t_scale'])
        # APC Net Key Parameter
        self.lyap_switch_threshold = 0.45
        self.lyap_old = 10
        self.switch_control = False
        # Config Local Net
        self.apcNet_local = DirectSiamese()
        self.apcNet_local.model_fld = 'direct_siamese_e2e_zoom_01'
        self.apcNet_local.load_ckpt(load_best=False)
        # Config Global Net
        self.apcNet_global = DirectSiamese()
        self.apcNet_global.model_fld = 'direct_siamese_e2e_01'
        self.apcNet_global.load_ckpt(load_best=False)
        # Init msg for publishing
        self.joint_traj = JointTrajectory()
        self.joint_traj.joint_names = ['shoulder_pan_joint',
                                       'shoulder_lift_joint',
                                       'elbow_joint', 'wrist_1_joint',
                                       'wrist_2_joint', 'wrist_3_joint']
        self.joint_traj.points = [
            JointTrajectoryPoint(positions=[0] * 6, velocities=[0] * 6,
                                 time_from_start=rospy.Duration(0.1))]
        self.vel_ctrl_old = np.zeros([6, 1])
        # Approximated Synchronised Topics
        self.image_sub = message_filters.Subscriber(
            "/camera/image_color", Image)
        self.joint_sub = message_filters.Subscriber(
            "/joint_states", JointState)
        self.synced_msgs = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.joint_sub], 10, 0.1)
        self.synced_msgs.registerCallback(self.image_callback)
        self.joint_command_pub = rospy.Publisher("/ur_driver/joint_speed",
                                                 JointTrajectory, queue_size=10)

    def image_callback(self, image_msg, joint_msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(image_msg, "rgb8")
        except CvBridgeError, e:
            print(e)
        else:
            joint_config = self.joint_msg2config(joint_msg)
            vel_command = self.infer_vel_command(cv2_img, joint_config)
            self.joint_traj.points = [
                JointTrajectoryPoint(positions=[0] * 6, velocities=vel_command,
                                     time_from_start=rospy.Duration(0.1))]

    def infer_vel_command(self, cv2_img, joint_config):
        # Init Zoom Mechanism
        img_global = self.cv2_img2torch(cv2_img)
        # Hot Swap
        lyap_global, pd_global = self.infer_partials(img_global, joint_config,
                                                     self.apcNet_global)
        zoom_in = ZoomAtHand()
        img_local = zoom_in.get_zoomed_image(cv2_img, joint_config)
        cv2.imshow('test', cv2.cvtColor(img_local, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        img_local = self.cv2_img2torch(img_local)
        lyap_local, pd_local = self.infer_partials(img_local, joint_config,
                                                   self.apcNet_local)
        lyap = lyap_global
        pd = pd_global
        print ("Lyap Inferred Global: %.5f" % lyap_global)
        print ("Lyap Inferred Local: %.5f" % lyap_local)
        if lyap_global < self.lyap_switch_threshold and not self.switch_control:
            self.switch_control = True

        if self.switch_control:
            print('Using local controller')
            lyap = lyap_local
            pd = pd_local
        # if lyap_global < self.lyap_switch_threshold and lyap_local>0:
        #     lyap = lyap_local
        #     pd = pd_local
        # else:
        #     lyap = lyap_global
        #     pd = pd_global
        # Smoothing Vel Command
        mu = 0.7
        smoothed_lyap = mu * self.lyap_old + (1 - mu) * lyap
        self.lyap_old = smoothed_lyap
        vel_ctrl_temp = self.cmpt_vel_ctrl(lyap, pd)
        momentum = 0.1
        vel_ctrl = momentum * self.vel_ctrl_old + (
                    1 - momentum) * vel_ctrl_temp
        self.vel_ctrl_old = vel_ctrl
        # Condition Vel Controller Gain
        if lyap >= self.lyap_switch_threshold:
            phy = 1
        elif lyap < self.lyap_switch_threshold and lyap > 0.1:
            phy = 1
        else:
            phy = 0
        vel_command = -phy * vel_ctrl
        print ("Vel Controller:")
        print (vel_command)
        print ('Joint_Config')
        print (joint_config)
        print ('\n')
        print [round(lyap_global, 5)] + [round(lyap_local, 5)] + \
                                      map(float, list(joint_config.reshape(6)))
        return vel_command

    def infer_partials(self, img, joint_config, apcNet):
        ur5_kin_scaled = UR5Kin(self.t_scale)
        with torch.no_grad():
            apcNet.net = apcNet.net.eval()
            vp = ur5_kin_scaled.get_veced_joint_poses(joint_config)
            vp = self.np2torch(vp).cuda()
            vp = vp.unsqueeze(0)
            lyap, pd = apcNet.net.infer(img.cuda(), vp)
        return lyap, pd

    def cmpt_vel_ctrl(self, lyap, pd):
        if lyap > 0:
            phy = 0.5 * min(1 / lyap, 1)
            vel_ctrl = phy * pd
        else:
            vel_ctrl = 0.0 * pd
        return vel_ctrl.reshape((6, 1))

    ############
    # Tool Box #
    ############
    def joint_msg2config(self, joint_msg):
        joint_config = np.asarray(joint_msg.position)
        joint_config = np.around(joint_config, decimals=4)
        joint_config = joint_config.reshape((6, 1))
        return joint_config

    def cv2_img2torch(self, cv_img):
        # img = np.zeros(np.flipud(cv_img.shape))
        crop_size = cv_img.shape[0]
        preprocess = transforms.Compose([transforms.ToPILImage(),
                                         transforms.CenterCrop(crop_size),
                                         transforms.Resize((224, 224)),
                                         transforms.ToTensor(),
                                         transforms.Normalize(
                                             mean=[0.485, 0.456, 0.406],
                                             std=[0.229, 0.224, 0.225])])
        img = preprocess(cv_img)
        img = img.unsqueeze(0)
        return img

    def np2torch(self, v):
        v = torch.from_numpy(v)
        v = v.type(torch.FloatTensor)
        return v

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
    rospy.init_node("hot_swap_real", anonymous=True)
    vel_controller = APCNetHowSwapReal()
    while not rospy.is_shutdown():
        try:
            vel_controller.joint_command_pub.publish(vel_controller.joint_traj)
            rospy.sleep(0.008)  # 125 Hz
        except KeyboardInterrupt:
            print "Pressed \'Ctrl + c\', Shutting down ..."
            sys.exit()


if __name__ == "__main__":
    main(sys.argv)
