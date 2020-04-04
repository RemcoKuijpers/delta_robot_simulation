#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from kinematics import DeltaRobotKinematics
from math import radians, sin, degrees, cos
import time

class DeltaRobot(object):
    def __init__(self, name):
        rospy.init_node("control")
        self.commander = rospy.Publisher("/" + name + "/pos_cmd", Vector3, queue_size=10)
        self.kinematics = DeltaRobotKinematics()
    
    def moveMotors(self, m1, m2, m3):
        msg = Vector3()
        msg.x, msg.y, msg.z = -radians(m1), -radians(m2), -radians(m3)
        self.commander.publish(msg)

    def movePositionCartesian(self, x, y, z):
        m1, m2, m3 = self.kinematics.reverse(x,y,z)
        #print(m1,m2,m3)
        self.moveMotors(m1,m2,m3)

if __name__ == "__main__":
    robot = DeltaRobot("delta_robot")
    while not rospy.is_shutdown():
        robot.movePositionCartesian(300*sin(time.time()), 300*cos(time.time()), -700)