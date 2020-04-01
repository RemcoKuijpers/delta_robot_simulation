#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from math import radians
import os, rospkg
import numpy as np
import time

class RobotCommander(object):
    def __init__(self):
        rospy.init_node("robot_commander")
        rospack = rospkg.RosPack()
        self.t, self.a, self.b, self.c, self.d = np.loadtxt(os.path.join(rospack.get_path("delta_robot_simulation"), "csv", "angles.csv"), delimiter=",", unpack=True)
        self.dt = []
        self.i = 0
        self.getTimeDifference()
        self.convertToRadians()
        self.anglePub = rospy.Publisher("/delta_robot/pos_cmd", Vector3, queue_size=10)
        self.anglePub2 = rospy.Publisher("/delta_robot2/pos_cmd", Vector3, queue_size=10)
        self.anglePub_ee = rospy.Publisher("/delta_robot/ee_cmd", Float32, queue_size=10)
        self.anglePub_ee2 = rospy.Publisher("/delta_robot2/ee_cmd", Float32, queue_size=10)
        self.publish()

    def getTimeDifference(self):
        for x in range(len(self.t)):
            if x == len(self.t)-1:
                break
            self.dt.append(self.t[x+1] - self.t[x])

    def convertToRadians(self):
        for x in range(len(self.t)):
            self.a[x] = radians(self.a[x])
            self.b[x] = radians(self.b[x])
            self.c[x] = radians(self.c[x])
            self.d[x] = radians(self.d[x])

    def publish(self):
        msg = Vector3()
        ee_msg = Float32()
        for t, i in zip(self.dt, range(len(self.dt))):
            now = time.time()
            msg.x = -self.a[i]
            msg.y = -self.b[i]
            msg.z = -self.c[i]
            ee_msg = self.d[i]
            self.anglePub.publish(msg)
            self.anglePub2.publish(msg)
            self.anglePub_ee.publish(ee_msg)
            self.anglePub_ee2.publish(ee_msg)
            elapsed = time.time() - now
            time.sleep(t/1000000 - elapsed)

if __name__ == "__main__":
    c = RobotCommander()