#!/usr/bin/env python
import roslib;

roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi


class UR5Controller():
    def __init__(self):
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                            'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.home_pose = [3 * pi / 4, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0]
        self.buff_pose = [2.6, -1, pi / 2, -2.1, -pi / 2, 0]
        self.release_pose = [3.6, -1.24, 1.24, -pi / 2, -pi / 2, 0]
        self.client = actionlib.SimpleActionClient('follow_joint_trajectory',
                                                   FollowJointTrajectoryAction)
        print ("Waiting for server...")
        self.client.wait_for_server()
        print ("Connected to server")
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index + len("prefix': '"):(
                        index + len("prefix': '") + str(parameters)[index + len(
                    "prefix': '"):-1].find("'"))]
            for i, name in enumerate(self.joint_names):
                self.joint_names[i] = prefix + name

    def move2target_pose(self, target_pose, duration):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_names
        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0] * 6,
                                     time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=target_pose, velocities=[0] * 6,
                                     time_from_start=rospy.Duration(duration))]
            self.client.send_goal(g)
            result = self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise
        return result

    def home(self):
        status = self.move2target_pose(self.home_pose, duration=3.0)
        return status

    def move2rest_pose(self):
        self.move2target_pose(self.home_pose, 1.0)
        status = self.move2target_pose(self.release_pose, 3.0)
        return status


def main():
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        # inp = raw_input("Continue? y/n: ")[0]
        ur5_controller = UR5Controller()
        # if (inp == 'y'):
        ur5_controller.home()
        # else:
        #     print ("Halting program")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__':
    main()
