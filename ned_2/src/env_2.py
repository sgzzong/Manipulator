#!/usr/bin/env python
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

class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        robot = moveit_commander.RobotCommander() #robot 의 정보를 제공, 현재 state
        scene = moveit_commander.PlanningSceneInterface() #원격 인터페이스를 제공
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name) # move_group node로 동작을 계획하고,  실행 
        group_names = robot.get_group_names()

        self.pub = rospy.Publisher('/state',String, queue_size=1)
        self.sub = rospy.Subscriber('/state', String, self.callback)
        
        self.move_group = move_group
        self.joint_angle = [0,0,0,0,0,0]
        self.reward = 0
        self.target = [[0.2,0.3,0.4],[-0.2,-0.3,0.2],[0.1,0.1,0.3],[0.3,0.2,0.1]]
        

    def callback(self,msg):                                                                                   
        print(msg.data) #여기서 만약 data값이 들어오면 action 을 하도록 함, action이 끝나면 상태값과 보상값을 publish, reset값이 들어오면 reset되어야 함, 메세지 구조 다시 정의
        # 메세지 구조를 [명령], [값] 으로 나눠서 보내야됌, ros.spin()으로 env코드에서 pub,sub이 가능하게 하는 것이 첫번 째 목표 >> 예를 들어 agent 코드에서 action, 값을 pub하면, 
        # env코드에서 action을 실행하고 상태,보상 publish
        if msg.data[0] == "action":
            self.move_group.action(msg.data)
            self.get_joint()
            self.send(self.joint_angle,self.reward)
        
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
        print("==============================action===============================")
        for i in range(6):
            self.joint_angle[i] = self.Degree_to_Radian(angle)[i]
        self.move_group.go(self.joint_angle, wait=True)
        self.move_group.stop()

    def reset(self):
        print("===============================reset===============================")
        for i in range(6):
            self.joint_angle[i] = 0
        self.move_group.go(self.joint_angle, wait=True)
        self.move_group.stop()

    def get_joint(self):
        print("===============================state===============================")
        joint = self.move_group.get_current_joint_values()
        for i in range(6):
            joint[i] = round(self.Radian_to_Degree(joint)[i],3)
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
            data += str(i)
        self.pub.publish(data)
    
    def get_reward(self,target):
        self.reward = math.sqrt(abs((self.get_pose()[0]-target[0])**2 + (self.get_pose()[1]-target[1])**2 + (self.get_pose()[2]-target[2])**2 ))
        print("==============================reward===============================")
        print("target distance :", math.sqrt(abs((self.get_pose()[0]-target[0])**2 + (self.get_pose()[1]-target[1])**2 + (self.get_pose()[2]-target[2])**2 )))

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
    ned2_contoller = MoveGroupPythonIntefaceTutorial()
    rospy.spin()

if __name__ == '__main__':
    main()