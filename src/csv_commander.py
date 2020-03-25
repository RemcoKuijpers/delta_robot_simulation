#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from math import radians
import os, rospkg
import threading
import numpy as np

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
        self.angles = Vector3()
        self.startTimer(self.dt[0])

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

    def startTimer(self, us):
        self.timer = threading.Timer(us/1000000, self.publish)
        self.timer.start()

    def publish(self):
        self.timer.cancel()
        self.i += 1
        if self.i == len(self.t)-1:
            return
        self.startTimer(self.dt[self.i])

        self.angles.x = -self.a[self.i]
        self.angles.y = -self.b[self.i]
        self.angles.z = -self.c[self.i]
        self.anglePub.publish(self.angles)
        self.anglePub2.publish(self.angles)

if __name__ == "__main__":
    try:
        c = RobotCommander()
    except rospy.ROSException or KeyboardInterrupt:
        c.timer.cancel()