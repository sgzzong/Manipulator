#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander #Python Moveit interface를 사용하기 위한 모듈
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from math import *
import math
from random import uniform
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
angle = [[90,20,30,0,0,0],[60,10,20,40,40,0],[-60,-20,30,-40,-10,0],[-10, 20, -30,10,10,0]]

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

        self.sub = rospy.Subscriber('/action', String, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/state',String, queue_size=1)

        self.move_group = move_group
        self.joint_angle = [0,0,0,0,0,0]
        self.reward = 0
        self.target = [0.2,0.3,0.4]
        self.i = 0
        self.previous = ""
    def callback(self,msg):
        if self.previous != msg.data:
            command, sub_list = self.str_to_list(msg.data)
        # print(command)
        # print(sub_list)
            if command == "action":
                if len(sub_list) == 6:
                    self.action(sub_list)
                    self.get_joint()
                    self.send(self.joint_angle,self.reward)
                self.i +=1
                print(self.i)
            elif command == "reset":
                self.reset()
        self.previous = msg.data
        
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
    def str_to_list(self,string):
        sub_list = []
        sub_str = string.split()
        for i in range(len(sub_str)):
            if i == 0:
                command = sub_str[i]
            else:
                sub_list.append(float(sub_str[i]))
        return command, sub_list
    def action(self,angle):
        print("==============================action===============================")
        self.move_group.go(self.Degree_to_Radian(angle), wait=True)
        self.move_group.stop()

    def reset(self):
        print("===============================reset===============================")
        for i in range(6):
            self.joint_angle[i] = 0
        self.move_group.go(self.joint_angle, wait=True)
        self.move_group.stop()

    def get_joint(self):
        print("===============================state===============================")
        joint = self.Radian_to_Degree(self.move_group.get_current_joint_values())
        for i in range(6):
            joint[i] = round(joint[i],3)
        self.joint_angle = joint
        return joint

    def get_pose(self):
        pose = self.move_group.get_current_pose().pose
        pose_value = [round(pose.position.x,3),round(pose.position.y,3),round(pose.position.z,3)]
        return pose_value

    def send(self,joint_angle,reward):
        data = ""
        joint_angle.append(reward)
        for i in joint_angle:
            data += str(i) + " "
        self.pub.publish(data)
    
    def get_reward(self):
        self.reward = math.sqrt(abs((self.get_pose()[0]-self.target[0])**2 + (self.get_pose()[1]-self.target[1])**2 + (self.get_pose()[2]-self.target[2])**2 ))
        print("==============================reward===============================")
        #print("target distance :", math.sqrt(abs((self.get_pose()[0]-self.target[0])**2 + (self.get_pose()[1]-self.target[1])**2 + (self.get_pose()[2]-self.target[2])**2 )) )
        return self.reward

    def target_reset(self,target):
        state_msg = ModelState()
        state_msg.model_name = 'cube_red'
        state_msg.pose.position.x = target[0] 
        state_msg.pose.position.y = target[1]
        state_msg.pose.position.z = target[2]
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        for i in range(20):
            set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            resp = set_state(state_msg) 
def main():
    global angle 
    ned2_contoller = Ned2_control()
    ned2_contoller.reset()
    # ned2_contoller.action(angle[0])
    # ned2_contoller.get_joint()
    # #ned2_contoller.reward(ned2_contoller.target[0])
    # ned2_contoller.send(ned2_contoller.joint_angle, ned2_contoller.reward)
    #rospy.spin()
if __name__ == '__main__':
    main()