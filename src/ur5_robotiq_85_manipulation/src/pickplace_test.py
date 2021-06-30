#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander, Grasp
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion


_frame_id = "base_link"
_arm_group_name = "arm"
_gripper_group_name = "hand"
_joint_gripper = "robotiq_85_left_knuckle_joint"
_grasps = []


def _open_gripper(posture):
    global _grasps
    global _frame_id
    global _arm_group_name
    global _joint_gripper
    global _gripper_group_name

    # Add finger joint of the robot
    posture.joint_names.append(_joint_gripper)

    # Set the gripper open
    point_ = JointTrajectoryPoint()
    point_.positions.append(0.01)
    posture.points.append(point_)


def _close_gripper(posture):
    global _grasps
    global _frame_id
    global _arm_group_name
    global _joint_gripper
    global _gripper_group_name

    # Add finger joint of the robot
    posture.joint_names.append(_joint_gripper)

    # Set the gripper closed
    point_ = JointTrajectoryPoint()
    point_.positions.append(0.01)
    posture.points.append(point_)

def _pick(_move_group):
    global _grasps
    global _frame_id
    global _arm_group_name
    global _joint_gripper
    global _gripper_group_name

    grasp_ = Grasp()
    _grasps.append(grasp_)

    grasp_.grasp_pose.header.frame_id = _frame_id
    
    q_ = quaternion_from_euler(-pi/2, 0.0, 0.0)
    # grasp_.grasp_pose.pose.orientation = Quaternion(-0.70661, 0.00779, 0.0082, 0.707512)
    grasp_.grasp_pose.pose.orientation = Quaternion(*q_)
    grasp_.grasp_pose.pose.position.x = 0.0002
    grasp_.grasp_pose.pose.position.y = 0.65
    grasp_.grasp_pose.pose.position.z = 0.09
    grasp_.grasp_pose.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

    # Pre-grasp approach
    grasp_.pre_grasp_approach.direction.header.frame_id = _frame_id
    grasp_.pre_grasp_approach.direction.vector.y = 1.0
    grasp_.pre_grasp_approach.min_distance = 0.075
    grasp_.pre_grasp_approach.desired_distance = 0.2
    grasp_.pre_grasp_approach.direction.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

    # Post-grasp retreat
    grasp_.post_grasp_retreat.direction.header.frame_id = _frame_id
    grasp_.post_grasp_retreat.direction.vector.z = 1.0
    grasp_.post_grasp_retreat.min_distance = 0.1
    grasp_.post_grasp_retreat.desired_distance = 0.2
    grasp_.post_grasp_retreat.direction.header.stamp = rospy.Time.now() + rospy.Duration(0.01)

    # Set the posture of end-effector before grasp
    _open_gripper(grasp_.pre_grasp_posture)

    # Set the posture of end-effector during grasp
    _close_gripper(grasp_.grasp_posture)

    # Table 1 is the supporting surface for the grasp object
    _move_group.set_support_surface_name("table1")

    # Pick up the object with grasps
    plan = _move_group.pick("coke_can", _grasps)
    
    print(_move_group.get_active_joints())

def _place():
    pass


#Initialize ROS node and moveit objects
rospy.init_node("panda_pick_and_place")
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = _arm_group_name
move_group = moveit_commander.MoveGroupCommander(group_name)
rospy.sleep(2)

# # Pick object
# _pick(move_group)
# rospy.sleep(1)

# q_ = quaternion_from_euler(-pi / 2, -pi / 4, -pi / 2)
# eu = euler_from_quaternion([0.0, 0.70719, -0.707002, 0.0])
# print(eu)

def get_rekt():
    start = rospy.get_time()
    seconds = rospy.get_time()
    timeout = 2.0 # in seconds

    # Loop until the objects placed are on the scene or the time runs out
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if models are on scene
        is_known_table1 = 'table1' in scene.get_known_object_names()
        is_known_table2 = 'table2' in scene.get_known_object_names()
        is_known_coke_can = 'coke_can' in scene.get_known_object_names()

        if is_known_table1 and is_known_table2 and is_known_coke_can:
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If all objects don't appear until the timeout
    return False

print(get_rekt())