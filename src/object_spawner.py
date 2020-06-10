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
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        rospack = rospkg.RosPack()
        #self.ids = []
        #self.dids = []
        with open(os.path.join(rospack.get_path("delta_robot_simulation"), "urdf", "object.urdf"), "r") as f:
            self.model = f.read()

    def spawnPotato(self, id, x_position, y_position):
        item_name = "potato_{0}".format(id)
        quat = tf.transformations.quaternion_from_euler(0,0,random.uniform(0,2*pi))
        pose = Pose(Point(x=x_position, y=y_position, z=1.1),   Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        self.spawn(item_name, self.model, "", pose, "world")

    def spawnManyPotatos(self, n):
        for id in range(1,n+1):
            self.spawnPotato(id, 0, 0)

    def deletePotato(self, id):
        item_name = "potato_{0}".format(id)
        self.delete(item_name)

    def update(self, i, x_position, y_position):
            msg = ModelState()
            msg.model_name = "potato_{0}".format(i)
            msg.pose.position.y = y_position
            msg.pose.position.x = x_position
            msg.pose.position.z = 1
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
            self.pub.publish(msg)    
        
    #def updatePose(self, i, x_position, y_position):
    #    if y_position > 1.4 and i not in self.dids:
    #        self.deletePotato(i)
    #        self.dids.append(i)
    #    elif i not in self.ids and i not in self.dids:
    #        self.spawnPotato(i, x_position, y_position)
    #        self.ids.append(i)
    #    elif i not in self.dids:
    #        self.update(i, x_position, y_position)

    def updatePoseNew(self, i, x_position, y_position, active):
        d = i/float(active)
        if d > 1:
            i = i - active*int(d)+1
        if i != 0:
            self.update(i, x_position, y_position)