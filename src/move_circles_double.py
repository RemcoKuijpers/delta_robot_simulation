#!/usr/bin/env python

import rospy
import time
from math import sin, cos
from delta_robot import DeltaRobot

if __name__ == "__main__":
    robot = DeltaRobot("delta_robot")
    robot2 = DeltaRobot("delta_robot2")
    z = -700
    while not rospy.is_shutdown():
        x = 100*sin(time.time()*2)
        y = 100*cos(time.time()*2)
        x2 = 200*sin(time.time()*4)
        y2 = 200*cos(time.time()*4)
        robot.movePositionCartesian(x,y,z)
        robot2.movePositionCartesian(x2,y2,z)