#!/usr/bin/env python

import rospy
import rospkg
import socket
import re
import tf
import os
from geometry_msgs.msg import Pose, Quaternion, Point
from gazebo_msgs.srv import SpawnModel
from math import radians

class RobotSpawner():
    def __init__(self):
        rospy.init_node("delta_robot_spawner")
        rospack = rospkg.RosPack()
        TCP_IP = rospy.get_param("/robot_controller_ip_address")
        TCP_PORT = 5007
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        with open(os.path.join(rospack.get_path("delta_robot_simulation"), "urdf", "codian.sdf"), "r") as f:
            self.model = f.read()
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((TCP_IP, TCP_PORT))

    def spawn_robot(self, id, x, y, z, rx, ry, rz):
        item_name = "delta_robot{0}".format(id)
        quat = tf.transformations.quaternion_from_euler(radians(rx), radians(ry), radians(rz))
        pose = Pose(Point(x=x-0.007, y=y+0.008, z=-z-1),   Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
        print(pose)
        self.spawn(item_name, self.model, "", pose, "world")

    def auto_spawn_robot(self):
        data = self.s.recvfrom(81)
        data = re.findall(r'-?\d+', data[0])
        print(data)
        if data != []:
            for i in range(1, int(data[0])+1):
                self.spawn_robot(i, float(data[1+(i-1)*9])/1000, float(data[3+((i-1)*9)])/1000, float(data[5+(i-1)*9])/1000, float(data[7+(i-1)*9])/100, float(data[8+(i-1)*9])/100, float(data[9+(i-1)*9])/100-90-(i-1)*120)
                break

if __name__ == "__main__":
    rs = RobotSpawner()
    rs.auto_spawn_robot()