#!/usr/bin/env python

# Imports
from niryo_robot_python_ros_wrapper import *
import rospy

# Initializing ROS node
rospy.init_node('niryo_ned_example_python_ros_wrapper')

# Connecting to the ROS Wrapper & calibrating if needed
niryo_robot = NiryoRosWrapper()
niryo_robot.calibrate_auto()

# Moving joint
while(1):
    niryo_robot.move_joints(0, 0, 0, 0, 0, 0)
    niryo_robot.move_joints(0.1, -0.2, 0.0, 1.1, -0.5, 0.2)