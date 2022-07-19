#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import random
from math import pi
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
orient = []
path = [[0.4,0.2,0.3],[0.25,0.2,0.3],[0.25,-0.2,0.3],
        [0.4,-0.2,0.2],[0.4,0.2,0.2],[0.25,0.2,0.2],[0.25,-0.2,0.2],[0.4,-0.2,0.3]]

pathv = [[0.4,0.2,0.3],[0.4,0.2,0.2],[0.25,0.2,0.3],[0.25,0.2,0.2],[0.25,-0.2,0.3],
        [0.25,-0.2,0.2],[0.4,-0.2,0.2],[0.4,-0.2,0.3]]
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
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.display_trajectory_publisher = display_trajectory_publisher
    def go_home(self):
        global orient
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/2
        joint_goal[3] = -pi/2
        joint_goal[4] = -pi/2
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_pose = self.move_group.get_current_pose().pose
        orient = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    def go_to_pose_goal(self,x,y,z):
        global orient
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = orient[0]
        pose_goal.orientation.y = orient[1]
        pose_goal.orientation.z = orient[2]
        pose_goal.orientation.w = orient[3]

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z #static

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    def plan_cartesian_path(self,x,y,z,scale=1):
        global orient
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        wpose.orientation.x = orient[0]
        wpose.orientation.y = orient[1]
        wpose.orientation.z = orient[2]
        wpose.orientation.w = orient[3]
        
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 
        )
        move_group.execute(plan, wait=True)
        move_group.stop()

    def random_reach(self):
        ran = [round(random.uniform(0.4, 0.25),2), round(random.uniform(-0.2, 0.2),2), round(random.uniform(0.2, 0.3),2)]
        print("move to (%.3f, %.3f, %.3f)"%(ran[0],ran[1],ran[2]))
        self.merge(ran)

    def merge(self,goal):
        move_group = self.move_group
        current_pose = self.move_group.get_current_pose().pose
        # print(goal[0],goal[1],goal[2])
        if abs(current_pose.position.x - goal[0]) < 0.01 and abs(current_pose.position.y - goal[1]) < 0.01:
          self.plan_cartesian_path(goal[0],goal[1],goal[2])
          print("use MoveL")
        else:
          self.go_to_pose_goal(goal[0],goal[1],goal[2])
          print("use MoveJ")
def main():    
    global path, pathv
    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.sleep(1)
    tutorial.go_home()
    i = 0
    while(True):
      print("=============================================")
      x = raw_input("h : Reset!  m : merge!  r : random!  e: Exit\n")
      if x == 'h':
        print("arrived home")
        tutorial.go_home()
      if x == 'm':
        rospy.sleep(10)
        for k in range(8):
          tutorial.merge(pathv[k])
          rospy.sleep(1)
      if x == 'r':
        rospy.sleep(10)
        for k in range(8):
          tutorial.random_reach()
          rospy.sleep(1)
      if x == 'e':
        print("Exit")
        break
      else:
        print("@@Re-enter@@")
if __name__ == '__main__':
  main()