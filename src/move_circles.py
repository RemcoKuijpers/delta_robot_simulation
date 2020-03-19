#!/usr/bin/env python

from kinematics import DeltaRobotKinematics, DeltaPositionError
import rospy
import time
from geometry_msgs.msg import Vector3
from math import sin, cos, radians

if __name__ == "__main__":
    rospy.init_node("circle_publisher")
    pub = rospy.Publisher("/delta_robot/pos_cmd", Vector3, queue_size=10)
    a = Vector3()
    kin = DeltaRobotKinematics()
    z = -800
    while not rospy.is_shutdown():
        try:
            x = 300*sin(time.time()*2)
            y = 300*cos(time.time()*4)
            z = 100*cos(time.time()*2)-900
            m1, m2, m3 = kin.reverse(x,y,z)
            a.x, a.y, a.z = radians(m1), radians(m2), radians(m3)
            pub.publish(a)
        except rospy.ROSException or KeyboardInterrupt:
            break