#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a C-Model gripper.

This serves as an example for publishing messages on the 'CModelRobotOutput' topic using the 'CModel_robot_output' msg type for sending commands to a C-Model gripper.
"""

import roslib;

roslib.load_manifest('robotiq_c_model_control')
import rospy
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg


class GripperController():
    def __init__(self):
        self.command = outputMsg.CModel_robot_output()
        self.pub = rospy.Publisher('CModelRobotOutput',
                                   outputMsg.CModel_robot_output, queue_size=10)
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 205
        self.command.rFR = 150

    def gen_command(self, char):
        """Update the command according to the character entered by the user."""
        if char == 'a':
            self.command = outputMsg.CModel_robot_output()
            self.command.rACT = 1
            self.command.rGTO = 1
            self.command.rSP = 255
            self.command.rFR = 150

        if char == 'r':
            self.command = outputMsg.CModel_robot_output()
            self.command.rACT = 0

        if char == 'c':
            self.command.rPR = 255

        if char == 'o':
            self.command.rPR = 0

        # If the self.command entered is a int, assign this value to rPRA
        try:
            self.command.rPR = int(char)
            self.command.rPR = min(self.command.rPR, 255)
            self.command.rPR = max(self.command.rPR, 0)
        except ValueError:
            pass

        if char == 'f':
            self.command.rSP += 25
            self.command.rSP = min(self.command.rSP, 255)

        if char == 'l':
            self.command.rSP -= 25
            self.command.rSP = max(self.command.rSP, 0)

        if char == 'i':
            self.command.rFR += 25
            self.command.rFR = min(self.command.rFR, 255)

        if char == 'd':
            self.command.rFR -= 25
            self.command.rFR = max(self.command.rFR, 0)

    def execute_command(self):
        for i in range(0, 5):
            self.pub.publish(self.command)
            rospy.sleep(0.1)

    def close(self):
        self.gen_command(100)
        self.execute_command()

    def open(self):
        self.gen_command('o')
        self.execute_command()


if __name__ == '__main__':
    rospy.init_node("gripper_test", anonymous=True)
    gripper_controller = GripperController()
    gripper_controller.close()
