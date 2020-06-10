#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from kinematics import reverse, forward
from math import radians, sin, degrees, cos
import time

class DeltaRobot(object):
    def __init__(self, name):
        rospy.init_node("robot_commander")
        self.commander = rospy.Publisher("/" + name + "/pos_cmd", Vector3, queue_size=10)
        self.ee = rospy.Publisher("/" + name + "/ee_cmd", Float32, queue_size=10)
    
    def moveMotors(self, m1, m2, m3, m4):
        msg = Vector3()
        msg2 = Float32()
        msg.x, msg.y, msg.z = -radians(m1), -radians(m3), -radians(m2)
        msg2.data = radians(m4)
        self.commander.publish(msg)
        self.ee.publish(msg2)

    def movePositionCartesian(self, x, y, z, aux):
        [m1, m2, m3] = reverse(x,y,z)
        self.moveMotors(degrees(m1),degrees(m2),degrees(m3),aux)

if __name__ == "__main__":
    robot = DeltaRobot("delta_robot")
    while not rospy.is_shutdown():
        robot.movePositionCartesian(300*sin(time.time()), 300*cos(time.time()), -700)