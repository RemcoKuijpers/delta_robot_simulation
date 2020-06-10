#!/usr/bin/env python

import rospy
import time
from math import sin, cos
from delta_robot import DeltaRobot

if __name__ == "__main__":
    robot1 = DeltaRobot("delta_robot")
    robot2 = DeltaRobot("delta_robot2")
    z = -800
    while not rospy.is_shutdown():
        try:
            #x = int(input('X:'))
            #y = int(input('Y:'))
            #z = int(input('Z:'))
            x = 200*cos(time.time()*4)
            y = 200*sin(time.time()*4)
            aux = 720*sin(time.time())
            robot1.movePositionCartesian(x,y,z,aux)
            robot2.movePositionCartesian(x,y,z,aux)
        except rospy.ROSException or KeyboardInterrupt:
            break