#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander #Python Moveit interface를 사용하기 위한 모듈
import moveit_msgs.msg
import geometry_msgs.msg
from math import *
import math
from random import uniform
from moveit_commander.conversions import pose_to_list

class Ned2_control(object):
    def __init__(self):
        super(Ned2_control, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        robot = moveit_commander.RobotCommander() #robot 의 정보를 제공, 현재 state
        scene = moveit_commander.PlanningSceneInterface() #원격 인터페이스를 제공
        group_name = "ned2"
        move_group = moveit_commander.MoveGroupCommander(group_name) # move_group node로 동작을 계획하고,  실행 
        group_names = robot.get_group_names()

        self.move_group = move_group
        self.target = [0.2,0.3,0.4]

    def Degree_to_Radian(self,Dinput):
        Radian_list = []
        for i in Dinput:
            Radian_list.append(i* (math.pi/180.0))
        return Radian_list

    def Radian_to_Degree(self,Rinput):
        Degree_list = []
        for i in Rinput:
            Degree_list.append(i* (180.0/math.pi))
        return Degree_list

    def action(self,angle):
        print("action : %f %f %f %f %f %f " % (angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]))
        self.move_group.go(self.Degree_to_Radian(angle), wait=False)
        self.move_group.stop()

    def reset(self):
        print("Go To Home pose")
        self.move_group.go([0,0,0,0,0,0], wait=True)
        self.move_group.stop()

    def state(self): #joint 6축 각도
        joint = self.Radian_to_Degree(self.move_group.get_current_joint_values())
        for i in range(6):
            joint[i] = round(joint[i],3)
        return joint

    def get_pose(self):
        pose = self.move_group.get_current_pose().pose
        pose_value = [round(pose.position.x,3),round(pose.position.y,3),round(pose.position.z,3)]
        return pose_value
    
    def reward(self):
        reward = math.sqrt(abs((self.get_pose()[0]-self.target[0])**2 + (self.get_pose()[1]-self.target[1])**2 + (self.get_pose()[2]-self.target[2])**2 ))
        #print("target distance :", math.sqrt(abs((self.get_pose()[0]-self.target[0])**2 + (self.get_pose()[1]-self.target[1])**2 + (self.get_pose()[2]-self.target[2])**2 )) )
        return reward
    