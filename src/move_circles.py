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
            x = 0#100*sin(time.time()*2)
            y = 0#100*cos(time.time()*2)
            robot.movePositionCartesian(x,y,z)
        except rospy.ROSException or KeyboardInterrupt:
            break