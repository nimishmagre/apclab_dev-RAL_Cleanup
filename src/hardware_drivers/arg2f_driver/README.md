# ARG2F Hardware Setup Guide
This document is written while putting the end-effector (arg2f140 robotiq) onto UR5.
## TCP Configuration:
TCP stands for Tool Centre Point. It is the point at the end of the robot arm that gives a characteristic point on the robot's tool. it is this point that is visualised on the graphics tab. This configuration process tells UR5 the specification
of the EE including Tool Centre of Mass, Mass, Tool Centre Point to make the tool a part of UR5 instead of treating the weight of EE as external applied force.
- All the hardware specifications can be found on [Robotiq Official Website, Instruction Manual, Section 6.](https://robotiq.com/support/2-finger-adaptive-robot-gripper)
- **TCP Config** is the corresponding setting on Polyscope.

## Control Gripper via USB
The current controller is modified version from **indigo branch/robotiq_c_model_control** [ros-industrial/robotiq](https://github.com/ros-industrial/robotiq). This [ROS package tutorial for the 2-Finger tutorial](https://dof.robotiq.com/discussion/199/ros-package-tutorial-for-the-2-finger) offers a good guideline.

Robotiq C Model is a 2-finger gripper series.
- Prerequisites: ` $ rosdep install robotiq_modbus_tcp `
- Configuring the Serial Port:
```
$ dmesg | grep tty
$ sudo chmod 777 /dev/ttyUSB0
```
- Starting the C-Model Node:
```
$ roscore # If a roscore has already started, ignore
$ rosrun robotiq_c_model_control CModelRtuNode.py /dev/ttyUSB0
# Another Terminal:
$ rosrun robotiq_c_model_control CModelSimpleController.py
```
  The driver listens for messages on "CModelRobotOutput" using the "SModel_robot_output" msg type.
The messages are interpreted and commands are sent to the Gripper accordingly.

  The "CModel_robot_output" msg type is simply composed of the robot output variables described in the Robotiq C-Model user manual. The simple controller node can therefore be modified to send
custom commands to the Gripper.
- Run the C-Model Status Listener Node:  `$ rosrun robotiq_c_model_control CModelStatusListener.py`
