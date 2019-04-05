#!/usr/bin/env python
import sys
import rospkg
import os
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path('ur_description'), 'ur5_kin'))
sys.path.append(os.path.join(rospack.get_path('apclab_dset_collector'), 'scripts'))
sys.path.append(os.path.join(rospack.get_path('apclab_control'), 'scripts/lyap_funcs'))
# print (os.path.join(rospack.get_path('apclab_control'), 'scripts/lyap_funcs'))
sys.path.append(os.path.expanduser('~/apcNet_dev/apcNet_Siam'))
import time
import rospy
import cv2
import math
import numpy as np
import torch
import message_filters
from pose_msg_converter import *
from ur5_kin_scaled import UR5Kin
from lyap_eh import LyapEH
from apcNet_siam_vgg13 import APCNetSiam
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetLinkState
from torchvision import transforms
import Tkinter as tk
import PIL.Image, PIL.ImageTk
from math import pi
from controller.arm2home_sim import arm2home_sim
from controller.move_cubes_sim import move_cubes_sim


bridge = CvBridge()
TABLE_HEIGHT = 0.7125


class APCController():

    def __init__(self, cnn_infer, t_scale):
        self.d_theta = 0.1
        self.t_scale = float(t_scale)
        self.grasp_offset = 0.05 # grasp offset along global Z axis
        self.window = tk.Tk()
        self.window.title('APCNet Simulator Demo')
     # Create a canvas that can fit the above video source size
        self.canvas = tk.Canvas(self.window, width = 848, height = 480)
        self.canvas.pack()
         # Button that lets the user take a snapshot
        self.start_bt = tk.Button(self.window, text='Start', width=50, command=self.start_bt_func)
        self.start_bt.pack(anchor=tk.CENTER, expand=True)
        self.home_bt = tk.Button(self.window, text="Home", width=50, command=arm2home_sim)
        self.home_bt.pack(anchor=tk.CENTER, expand=True)
        self.shuffle_bt = tk.Button(self.window, text="Shuffle Cubes", width=50, command=move_cubes_sim)
        self.shuffle_bt.pack(anchor=tk.CENTER, expand=True)
        self.delay = 30
        self.cv2_img = None
        self.update()
        self.is_start = False
        if cnn_infer == "infer":
            self.apcNet = APCNetSiam()
            self.apcNet.model_fld = 's2r_test02_2048_dense'
            self.apcNet.load_ckpt(load_best=False)
        self.preprocess = transforms.Compose([transforms.ToPILImage(),
                                               transforms.CenterCrop(480),
                                               transforms.Resize((224, 224)),
                                               transforms.ToTensor(),
                                               transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                                                    std=[0.229, 0.224, 0.225])])
        self.ur5_kin = UR5Kin(self.t_scale)
        self.lyap_func = LyapEH()
        self.apcNet_infer = cnn_infer
        self.image_topic = "/camera/rgb/image_raw"
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        # Init msg for publishing
        self.joint_traj = JointTrajectory()
        self.joint_traj.joint_names = self.joint_names
        self.real_joint_traj = JointTrajectory()
        self.real_joint_traj.joint_names = self.joint_names
        self.old_time = time.time()
        self.lyap_vctrl_old = np.zeros([6,1])
        self.real_joint_traj.points = [JointTrajectoryPoint(positions = [0]*6,
                                    velocities= [0]*6,
                                    time_from_start=rospy.Duration(0.1))]
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



    def image_callback(self, image_msg, joint_msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = bridge.imgmsg_to_cv2(image_msg, "rgb8")
        except CvBridgeError, e:
            print(e)
        else:
            if self.is_start:
                q = self.joint_msg2q(joint_msg)
                X_H = self.ur5_kin.get_ee_pose(q)
                X_T = self.get_best_target(X_H)
                ############################################
                if self.apcNet_infer == 'infer':
                    img  = self.cv2_img2torch(cv2_img)
                    lyap, J = self.infer_J(img, q)
                    lyap_vctrl_temp = self.cmpt_lyap_vctrl(lyap, J)
                    J_GT = self.cmpt_analyt_J(q, X_T, X_H)
                    lyap_GT = self.lyap_func.cmpt_Lyap(X_H)
                    print ("Lyap Inferred: %.5f" % lyap)
                    print ("Lyap GT: %.5f" % lyap_GT)
                elif self.apcNet_infer == 'analyt':
                    J = self.cmpt_analyt_J(q, X_T, X_H)
                    lyap = self.lyap_func.cmpt_Lyap(X_H)
                    lyap_vctrl_temp = self.cmpt_lyap_vctrl(lyap, J)
                else:
                    print ("####### Warning !!! #######")
                    print('   Mode not supported!!! \nChoose \"infer\" or \"analyt\" ')
                    print ("###########################")
                    rospy.signal_shutdown('Quit')

                momentum = 0.9
                lyap_vctrl = momentum*self.lyap_vctrl_old+(1-momentum)*lyap_vctrl_temp
                self.lyap_vctrl_old = lyap_vctrl

                if lyap > 0.001:
                    phy = 2
                else:
                    phy = 0
                vel_commands = -phy*lyap_vctrl
                # test_vel = np.array([-0.2, 0, 0, 0, 0, 0])
                # publish controller
                dt = time.time() - self.old_time
                self.old_time = time.time()
                target_pose = q + vel_commands*dt
                print ("Vel Controller:")
                print (vel_commands)
                print ('Joint_Config')
                print (q)
                self.joint_traj.points = [JointTrajectoryPoint(positions = q,
                                            velocities= -vel_commands,
                                            time_from_start=rospy.Duration(0.1))]
                self.sim_joint_command_pub.publish(self.joint_traj)


    def scale_translation(self, X):
        X[0:3, 3] = self.t_scale*X[0:3, 3]
        return X

    def get_best_target(self, X_H):
        X_T1 = self.get_target_pose('cube1_link', 'world')
        X_T2 = self.get_target_pose('cube2_link', 'world')
        X_T3 = self.get_target_pose('cube3_link', 'world')
        X_T_candidates = [X_T1, X_T2, X_T3]
        X_Ts = []
        for X_T_temp in X_T_candidates:
            if X_T_temp[2,3]>TABLE_HEIGHT*self.t_scale:
                X_Ts.append(X_T_temp)
        lyap_buff = np.array([])
        for X_T in X_Ts:
            if X_T[2,3]>TABLE_HEIGHT:
                self.lyap_func.X_T = X_T
                lyap_temp = self.lyap_func.cmpt_Lyap(X_H)
                lyap_buff = np.append(lyap_buff, lyap_temp)
        X_T = X_Ts[np.argmin(lyap_buff)]
        return X_T

    def infer_J(self, img, q):
        with torch.no_grad():
            start_time = time.time()
            self.apcNet.net = self.apcNet.net.eval()
            vp1 = self.ur5_kin.get_veced_joint_poses(q)
            # compute joint 6D v2
            vp_batch = np.zeros((7,72))
            vp_batch[0, :] = vp1
            for i in range(6):
                d_joint_vec = np.zeros((6,1))
                d_joint_vec[i, 0] = self.d_theta
                vp_batch[i+1, :] = self.ur5_kin.get_veced_joint_poses(q+d_joint_vec)
            vp_batch = self.np2torch(vp_batch)
            vp_batch = vp_batch.cuda()
            inferred_lyap, inferred_J = self.apcNet.net.infer_J(img.cuda(), vp_batch, self.d_theta)
            # print(inferred_lyap, inferred_J)
            # compute Jacobian
            torch.cuda.synchronize()
            infer_time = time.time() - start_time
            print ("Infering Time:%.4fs" % infer_time)
        return inferred_lyap, inferred_J

    def cmpt_lyap_vctrl(self, lyap, J):
        J_pinv = np.linalg.pinv(J)
        lyap_vctrl = J_pinv*lyap
        return lyap_vctrl

    def cmpt_joint_limit_vctrl(self, J, q):
        J_pinv = np.linalg.pinv(J)
        J_star = np.eye(6)-J_pinv.dot(J)
        joint_limit_err = self.cmpt_joint_limit_err(q)
        joint_limit_vctrl = J_star.dot(joint_limit_err)
        return joint_limit_vctrl

    def cmpt_analyt_J(self, q, X_T, X_H):
        self.lyap_func.X_T = X_T
        analyt_J = np.zeros((1,6))
        for i in range(6):
            pd_X_H = self.ur5_kin.get_pd_X_H(q, i)
            analyt_J[0, i] = self.lyap_func.cmpt_pd_Lyap(X_H, pd_X_H)
        # print analyt_J
        return analyt_J

    ############
    # Tool Box #
    ############
    def joint_msg2q(self, joint_msg):
        q = np.asarray(joint_msg.actual.positions)
        q = np.around(q, decimals=4)
        q = q.reshape((6,1))
        return q

    def cv2_img2torch(self, cv_img):
        img = np.zeros(np.flipud(cv_img.shape))
        # for i in range(3):
        #     img[i, :, :] = cv_img[:, :, i]
        img = self.preprocess(cv_img)
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

    def get_target_pose(self, link_name, reference_frame):
        X_T = self.get_link_pose(link_name, reference_frame)
        X_T[2,3] = X_T[2,3] + self.grasp_offset
        X_T = self.scale_translation(X_T)
        return X_T

    def update(self):
        if self.cv2_img is not None:
            self.current_frame = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(self.cv2_img))
            self.canvas.create_image(0, 0, image = self.current_frame, anchor = tk.NW)
            self.canvas.update()
        self.window.after(self.delay, self.update)

    def start_bt_func(self):
        self.is_start = not self.is_start
        if self.start_bt['text'] == 'Start':
            print('Pause')
            self.start_bt['text'] = 'Pause'
        elif self.start_bt['text'] == 'Pause':
            print('Start')
            self.start_bt['text'] = 'Start'

########################################################
def main(args):
    rospy.init_node("apc_controller", anonymous=True)
    cnn_infer = args[1]
    vel_controller = APCController(cnn_infer, t_scale=6)
    vel_controller.window.mainloop()

    # while not rospy.is_shutdown():
        # try:
        #     vel_controller.sim_joint_command_pub.publish(vel_controller.joint_traj)
        #     rospy.sleep(0.008)
        # except KeyboardInterrupt:
        #     print "Shutting down APCController module"
        #     sys.exit()

if __name__ == "__main__":
    main(sys.argv)
