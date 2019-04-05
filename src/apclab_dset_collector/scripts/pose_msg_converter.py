#!/usr/bin/env python
import numpy as np
import tf

def pose_vec2msg(pose_vec):
    pose_msg= geometry_msgs.msg.Pose()
    pose_msg.position.x=pose_vec[0]
    pose_msg.position.y=pose_vec[1]
    pose_msg.position.z=pose_vec[2]
    pose_msg.orientation.x=pose_vec[3]
    pose_msg.orientation.y=pose_vec[4]
    pose_msg.orientation.z=pose_vec[5]
    pose_msg.orientation.w=pose_vec[6]
    return pose_msg

def pose_msg2vec(pose_msg):
    pose_vec=np.array([pose_msg.pose.position.x,
                         pose_msg.pose.position.y, pose_msg.pose.position.z,
                         pose_msg.pose.orientation.x,
                         pose_msg.pose.orientation.y,
                         pose_msg.pose.orientation.z,
                         pose_msg.pose.orientation.w])
    return pose_vec

def pose_msg2se3(pose_msg):
    t = np.array([pose_msg.pose.position.x,
                pose_msg.pose.position.y, pose_msg.pose.position.z])
    Q = np.array([pose_msg.pose.orientation.x,
                     pose_msg.pose.orientation.y,
                     pose_msg.pose.orientation.z,
                     pose_msg.pose.orientation.w])
    R = tf.transformations.quaternion_matrix(Q)
    I = tf.transformations.identity_matrix() # 4x4 Identity_matrix
    X = R+tf.transformations.translation_matrix(t)-I
    return X
