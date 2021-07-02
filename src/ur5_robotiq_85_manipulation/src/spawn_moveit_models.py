#!/usr/bin/python

import rospy
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler


class ModelSpawn:
    def __init__(self):
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()
        
        rospy.sleep(1)

        self.table1_name_ = "table1"
        self.table2_name_ = "table2"
        self.coke_can_name_ = "coke_can"
        
        
    def _spawn_models(self):
        retval = self._add_objects_on_scene()
        assert retval, "[AssertionError]: Timeout ran out. Objects not placed on scene."


    def _add_objects_on_scene(self):
        """
        Add 2 tables and a can to the scene

        @return: True if objects are placed, false if not
        """

        # Delete everything on the scene - no params mean everything
        self._scene.remove_world_object()
        # rospy.sleep(1)

        # Add objects to the scene
        # All objects need to be placed relative to the robot, hence the weird z-axis
        self._add_table(self.table1_name_, 0.0, 1.0, -(1.03)/2, quaternion_from_euler(0.0, 0.0, 0.0))
        self._add_table(self.table2_name_, 0.0, -1.0, -(1.03)/2, quaternion_from_euler(0.0, 0.0, 0.0))
        
        self._add_coke_can(self.coke_can_name_, 0.0, 0.75, 0.122/2.0, quaternion_from_euler(0.0, 0.0, 0.0))


        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 2.0 # in seconds
        
        # Loop until the objects placed are on the scene or the time runs out
        while (seconds - start < timeout) and not rospy.is_shutdown():
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


    def _add_table(self, name, x, y, z, q):
        """
        Create and place a table (box shape) in MoveIt scene 

        @param x: position on x-axis
        @param y: position on y-axis
        @param z: position on z-axis
        @param q: quaternion
        @param table_num: number of table placed on scene
        """
        # Create a pose for the table
        p_ = PoseStamped()
        p_.header.frame_id = 'base_link'
        p_.header.stamp = rospy.Time.now()

        p_.pose.position.x = x
        p_.pose.position.y = y
        p_.pose.position.z = z
        p_.pose.orientation = Quaternion(*q)

        # Table size is used from ur5_robotiq_85_manipulation/models/table/model.sdf
        box_size_ = (1.5, 0.8, 1.03)
        self._scene.add_box(name, p_, box_size_)


    def _add_coke_can(self, name, x, y, z, q):
        """
        Create and place a coke can (cylinder shape) in MoveIt scene 

        @param x: position on x-axis
        @param y: position on y-axis
        @param z: position on z-axis
        @param q: quaternion
        """
        
        # Create a pose for the coke can
        p_ = PoseStamped()
        p_.header.frame_id = "base_link"
        p_.header.stamp = rospy.Time.now()

        p_.pose.position.x = x
        p_.pose.position.y = y
        p_.pose.position.z = z
        p_.pose.orientation = Quaternion(*q)

        # Coke can size is used from ur5_robotiq_85_manipulation/models/coke_can/model.sdf (actually googled it)
        # diameter is 65 mm
        self._scene.add_cylinder(name, p_, height=0.122, radius=0.0325)



def main():
    rospy.init_node('spawn_moveit_models_node')

    model_spawner = ModelSpawn()
    model_spawner._spawn_models()

if __name__ == '__main__':
    main()