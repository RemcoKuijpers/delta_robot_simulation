#!/usr/bin/env python

import rospy 
import tf
import os
import rospkg
import time
import random
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point

class PotatoSpawner(object):
    def __init__(self):
        rospy.init_node("potato_spawner")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        rospack = rospkg.RosPack()
        with open(os.path.join(rospack.get_path("delta_robot_simulation"), "urdf", "object.urdf"), "r") as f:
            self.model = f.read()

    def spawnPotato(self, x_position, id):
        item_name = "potato_{0}".format(id)
        pose = Pose(Point(x=x_position, y=0, z=1.13),   Quaternion(x=0, y=0, z=0, w=1))
        self.spawn(item_name, self.model, "", pose, "world")

    def deletePotato(self, id):
        item_name = "potato_{0}".format(id)
        self.delete(item_name)

if __name__ == "__main__":
    s = PotatoSpawner()
    for i in range(11):
        s.spawnPotato(random.uniform(0, 1.71), i)
        time.sleep(1)
    for j in range(11):
        s.deletePotato(j)
        time.sleep(1)