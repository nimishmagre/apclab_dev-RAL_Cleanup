#!/usr/bin/env python
import numpy as np
# Notation: X represents homogeneous transformation, A_X_B represents coordinate
# mapping of frame B w.r.t A, A can be omitted if A is the world fixed frame.
# rigid-transformation is represented with triple index notation: AC_X_B. it
# represents the motion from frame C to B in A.


class LyapEC():
    """
    This Class defines the mathematical structure of computing the value of
    Lyapunov function.

    Considering a Static Scene, where only the robot arm can move, the only
    two properties of this class are X_T, X_C: the poses of target and camera
    respectively as they are static. the hand pose is considered as the input
    of instance methods.

    there are two main methods:
    - self.cmpt_err_mat(X_H) --> Lyapunov value (float)
    - self.cmpt_pd_Lyap(X_H, pd_X_H) --> pd Lyap wrt a joint angle

    """
    def __init__(self):
        self.X_T = None
        self.X_C = None

    # This function computes the Error function CH_E_T. E, C, H, T represents Error,
    # Camera, Hand (end-effector), Target respectively.

    def cmpt_err_mat(self, X_H):
        C_X_A = np.linalg.inv(self.X_C)
        H_X_A = np.linalg.inv(X_H)
        CH_E_T = C_X_A.dot(self.X_T).dot(H_X_A).dot(self.X_C)
        return CH_E_T

    def cmpt_Lyap(self, X_H):
        """
        :param X_H: Hand Pose 4x4
        :return: Lyapunov value at given setup.
        """
        CH_E_T = self.cmpt_err_mat(X_H)
        I = np.eye(4)
        Lyap = np.trace((CH_E_T-I).dot(np.transpose(CH_E_T-I)))
        return Lyap

    def cmpt_analyt_pd_err_mat(self, X_H, pd_X_H):
        C_X_A = np.linalg.inv(self.X_C)
        H_X_A = np.linalg.inv(X_H)
        pd_CH_E_T = -C_X_A.dot(self.X_T).dot(H_X_A).dot(pd_X_H).dot(H_X_A).dot(self.X_C)
        return pd_CH_E_T

    def cmpt_pd_Lyap(self, X_H, pd_X_H):
        """
        :param X_H: hand pose
        :param pd_X_H: partial derivative of hand pose wrt a specific joint angle
        :return: partial derivative of Lyap wrt a specific joint angle
        """
        CH_E_T = self.cmpt_err_mat(X_H)
        pd_CH_E_T = self.cmpt_analyt_pd_err_mat(X_H, pd_X_H)
        I = np.eye(4)
        pd_Lyap = 2*np.trace(np.transpose(CH_E_T-I).dot(pd_CH_E_T))
        return pd_Lyap
