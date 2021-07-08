#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface, RobotCommander, Grasp
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from tf.transformations import quaternion_from_euler

class PickAndPlace:
    def __init__(self):
    
        self._robot = RobotCommander()
        self._scene = PlanningSceneInterface()

        self._arm_group_name = rospy.get_param("~group_arm", default="arm")
        self._gripper_group_name = rospy.get_param("~group_gripper", default="hand")
        self._timeout = rospy.get_param("~timeout", default="2.0")

        self._joint_gripper = "robotiq_85_left_knuckle_joint"  # only one is enough, others are passive
        self._frame_id = "base_link"

        self._move_group = moveit_commander.MoveGroupCommander(self._arm_group_name)
        self._move_group.set_planning_time(5)

        self._move_group.set_max_velocity_scaling_factor(0.3)
        self._move_group.set_max_acceleration_scaling_factor(0.3)

        self._grasps = []


    def are_models_on_scene(self):
        """
        Checks if the models are on scene.
        @return bool: True if models are on scene, False if not
        """

        start = rospy.get_time()
        seconds = rospy.get_time()

        # Loop until the objects placed are on the scene or the time runs out
        while(seconds - start < self._timeout) and not rospy.is_shutdown():
            # Test if models are on scene
            is_known_table1 = 'table1' in self._scene.get_known_object_names()
            is_known_table2 = 'table2' in self._scene.get_known_object_names()
            is_known_coke_can = 'coke_can' in self._scene.get_known_object_names()

            if is_known_table1 and is_known_table2 and is_known_coke_can:
                return True
            
            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        
        # If all objects don't appear until the timeout
        return False


    def _open_gripper(self, posture):
        """
        Defines posture for open gripper.
        @param posture: posture in which the open gripper lies
        """

        # Add finger joint of the robot
        posture.joint_names.append(self._joint_gripper)

        # Set the gripper open
        point_ = JointTrajectoryPoint()
        point_.positions.append(0.01)
        point_.time_from_start = rospy.Duration(1.05)
        posture.points.append(point_)


    def _close_gripper(self, posture):
        """
        Defines posture for closed gripper.
        @param posture: posture in which the closed gripper lies
        """
        # Add finger joint of the robot
        posture.joint_names.append(self._joint_gripper)

        # Set the gripper closed
        point_ = JointTrajectoryPoint()
        point_.positions.append(0.425)
        point_.time_from_start = rospy.Duration(1.0)
        posture.points.append(point_)
    

    def _pick(self):
        """
        Set up grasp pose, pre-grasp approach and post-grasp 
        approach then pick up the can.
        """
        grasp_ = Grasp()
        self._grasps.append(grasp_)

        grasp_.grasp_pose.header.frame_id = self._frame_id
        
        q_ = quaternion_from_euler(-pi/2, 0.0, 0.0)
        grasp_.grasp_pose.pose.orientation = Quaternion(*q_)
        grasp_.grasp_pose.pose.position.x = 0.0
        grasp_.grasp_pose.pose.position.y = 0.616
        grasp_.grasp_pose.pose.position.z = 0.083

        # Pre-grasp approach
        grasp_.pre_grasp_approach.direction.header.frame_id = self._frame_id
        grasp_.pre_grasp_approach.direction.vector.y = 1.0
        grasp_.pre_grasp_approach.min_distance = 0.12
        grasp_.pre_grasp_approach.desired_distance = 0.3

        # Post-grasp retreat
        grasp_.post_grasp_retreat.direction.header.frame_id = self._frame_id
        grasp_.post_grasp_retreat.direction.vector.z = 1.0
        grasp_.post_grasp_retreat.min_distance = 0.2
        grasp_.post_grasp_retreat.desired_distance = 0.45

        # Set the posture of end-effector before grasp
        self._open_gripper(grasp_.pre_grasp_posture)

        # Set the posture of end-effector during grasp
        self._close_gripper(grasp_.grasp_posture)

        # Table 1 is the supporting surface for the grasp object
        self._move_group.set_support_surface_name('table1')

        # Pick up the object with grasps
        self._move_group.pick('coke_can', self._grasps)


    def _place(self):
        """
        Set up place pose, pre-place approach and post-place 
        approach then place the can.
        """

        point_ = moveit_commander.PlaceLocation()
        place_loc = []
        place_loc.append(point_)

        # Set the location pose
        point_.place_pose.header.frame_id = self._frame_id
        q_ = quaternion_from_euler(0.0, 0.0, -pi)
        point_.place_pose.pose.orientation = Quaternion(*q_)

        # The pose is the exact location of the center of the object
        point_.place_pose.pose.position.x = 0
        point_.place_pose.pose.position.y = -0.67
        point_.place_pose.pose.position.z = 0.07

        # Pre-place approach
        point_.pre_place_approach.direction.header.frame_id = self._frame_id
        point_.pre_place_approach.direction.vector.z = -1.0
        point_.pre_place_approach.min_distance = 0.1
        point_.pre_place_approach.desired_distance = 0.2

        # Post-grasp retreat
        point_.post_place_retreat.direction.header.frame_id = self._frame_id
        point_.post_place_retreat.direction.vector.y = 1.0
        point_.post_place_retreat.min_distance = 0.07
        point_.post_place_retreat.desired_distance = 0.2

        # Set the posture after placing the object
        self._open_gripper(point_.post_place_posture)

        # Table2 is support surface
        self._move_group.set_support_surface_name("table2")

        # Call "place" to place the object using the set locations
        self._move_group.place("coke_can", place_loc)

    def performPickPlace(self):
        """
        Performs the pick and place pipeline with homing sequence.
        """
        rospy.sleep(2)
        self._pick()
        
        rospy.sleep(2)
        self._place()

    def go_home(self):
        """
        Position the robot home.
        """
        rospy.sleep(2) # preventive sleep 
        self._move_group.set_named_target("home")
        self._move_group.go(wait=True)


def main():

    rospy.init_node("pick_and_place_node", disable_signals=True)


    # Create the object and then wait for initialization
    pick_place_holder = PickAndPlace()
    rospy.sleep(2)

    # Check for models on scene
    models_on_scene = pick_place_holder.are_models_on_scene()
    
    if not models_on_scene: 
        rospy.signal_shutdown("No models on the scene. Shutting down.")
        return

    # Go to home position before the pick-and-place pipeline
    pick_place_holder.go_home()

    # Perform pick and place
    pick_place_holder.performPickPlace()

    # Rest at home position 
    pick_place_holder.go_home()



if __name__ == '__main__':
    main()