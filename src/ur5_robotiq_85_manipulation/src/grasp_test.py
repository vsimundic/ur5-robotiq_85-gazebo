#!/usr/bin/env python


import rospy
import moveit_commander

rospy.init_node('gripper_grasp')
gripper = moveit_commander.MoveGroupCommander("hand")
gripper.set_named_target('closed')
gripper.go(wait=True)
