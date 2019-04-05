#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import sys, select, termios, tty

msg = """
Control Robotiq-ARG-2F-140 Gripper
------------------------------------------------
Close: w
Open: S
Every key-press close or open the gripper
CTRL-C to quit
"""

Joint_Names = ['finger_joint', 'left_inner_finger_joint', \
'left_inner_knuckle_joint', 'right_inner_finger_joint', \
'right_inner_knuckle_joint', 'right_outer_knuckle_joint']

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('talker', anonymous=True)
    close_level = 0
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    print (msg)
    while not rospy.is_shutdown():
            finger_angle = close_level*(0.77/5)
            joint_traj =  JointTrajectory()
            # joint_traj.header = msg.header
            joint_traj.joint_names = Joint_Names
            joint_config = np.array([1.0, 1.0, -1.0, 1.0, -1.0, -1.0])*finger_angle
            joint_traj.points = [JointTrajectoryPoint(positions=joint_config, \
            velocities=[0]*6, time_from_start=rospy.Duration(3.0))]
            pub.publish(joint_traj)
            rospy.sleep(0.01)
        except KeyboardInterrupt:
            print "Shutting down"
            sys.exit()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
