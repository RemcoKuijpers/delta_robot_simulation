#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from kinematics import DeltaRobotKinematics, DeltaPositionError
from math import radians

class DeltaRobot(object):
    def __init__(self):
        rospy.init_node("delta_robot_control")
        self.commander = rospy.Publisher("/delta_robot/pos_cmd", Vector3, queue_size=10)
        self.kinematics = DeltaRobotKinematics()
    
    def moveMotors(self, m1, m2, m3):
        msg = Vector3()
        msg.x, msg.y, msg.z = radians(m1), radians(m2), radians(m3)
        self.commander.publish(msg)

    def movePositionCartesian(self, x, y, z):
        m1, m2, m3 = self.kinematics.reverse(x,y,z)
        self.moveMotors(m1, m2, m3)