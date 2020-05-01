#!/usr/bin/env python

import rospy 
import tf
import os
import rospkg
import time
import random
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Quaternion, Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from math import pi

class PotatoSpawner(object):
    def __init__(self):
        rospy.init_node("potato_spawner")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service('/gazebo/set_model_state')
        self.delete = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.set = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospack = rospkg.RosPack()
        self.ids = []
        with open(os.path.join(rospack.get_path("delta_robot_simulation"), "urdf", "object.urdf"), "r") as f:
            self.model = f.read()

    def spawnPotato(self, id, x_position, y_position):
        item_name = "potato_{0}".format(id)
        quat = tf.transformations.quaternion_from_euler(0,0,random.uniform(0,2*pi))
        pose = Pose(Point(x=x_position, y=y_position, z=1.1),   Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        self.spawn(item_name, self.model, "", pose, "world")

    def spawnManyPotatos(self, n):
        for id in range(n):
            self.spawnPotato(id, 0, 0)
            self.ids.append(id)

    def deletePotato(self, id):
        item_name = "potato_{0}".format(id)
        self.delete(item_name)
        
    def updatePose(self, id, x_position, y_position):
        if id not in self.ids:
            #self.spawnPotato(id, x_position, y_position)
            #self.ids.append(id)
            #print("Spawned potato {0}".format(id))
            pass
        else:
            msg = ModelState()
            msg.model_name = "potato_{0}".format(id)
            msg.pose.position.y = y_position
            msg.pose.position.x = x_position
            msg.pose.position.z = 1.1
            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 0
            msg.twist.linear.x = 0
            msg.twist.linear.y = 0
            msg.twist.linear.z = 0
            msg.twist.angular.x = 0
            msg.twist.angular.y = 0
            msg.twist.angular.z = 0
            msg.reference_frame = "world"
            self.set(msg)
            #print("Updated pose {0}".format(id))

if __name__ == "__main__":
    s = PotatoSpawner()
    for i in range(11):
        s.spawnPotato(random.uniform(0.1, 1.71), i)
        time.sleep(1)
    for j in range(11):
        s.deletePotato(j)
        time.sleep(1)