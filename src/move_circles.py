#!/usr/bin/env python

import rospy
import time
from math import sin, cos
from delta_robot import DeltaRobot

if __name__ == "__main__":
    robot = DeltaRobot("delta_robot")
    z = -700
    while not rospy.is_shutdown():
        try:
            x = 200*cos(time.time()*4)
            y = 200*sin(time.time()*4)
            aux = 720*sin(time.time())
            robot.movePositionCartesian(x,y,z,aux)
        except rospy.ROSException or KeyboardInterrupt:
            break