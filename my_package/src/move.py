#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)                                                      
        group_names = robot.get_group_names()
        self.move_group = move_group

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = pi/4
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def go_home(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/2
        joint_goal[3] = -pi/2
        joint_goal[4] = -pi/2
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        print("arrived home")
        move_group.stop()
        # current_pose = self.move_group.get_current_pose().pose
        # print(current_pose)

    def go_to_pose_goal(self,x,y):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -4.84552997928e-05
        pose_goal.orientation.y = 0.707116921794
        pose_goal.orientation.z = 0.000111932580228
        pose_goal.orientation.w = 0.707096629914

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = 0.4 #static

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
def main():
    tutorial = MoveGroupPythonIntefaceTutorial()
    tutorial.go_home()
    while(True):
      print("===============================================================")
      x = raw_input("g : 포인트로 이동      h : 홈 위치로 이동         e : 종료 \n")
      if x == 'g':
        a, b = map(eval, raw_input('X, Y좌표를 입력하세요. : ').split())
        print("(%.2f,%.2f,0.4) 좌표로 이동합니다."%(a,b))
        tutorial.go_to_pose_goal(a, b)
      if x == 'h':
        print("home 위치로 돌아갑니다.")
        tutorial.go_home()
      if x == 'e':
        print("종료되었습니다.")
        break
      else:
        print("please press the correct key")
if __name__ == '__main__':
  main()
