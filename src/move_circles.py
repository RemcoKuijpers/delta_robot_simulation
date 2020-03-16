#!/usr/bin/env python

from kinematics import DeltaRobotKinematics, DeltaPositionError
import rospy
from geometry_msgs.msg import Vector3
from math import sin, cos, radians

if __name__ == "__main__":
    rospy.init_node("Circle_publiosher")
    pub = rospy.Publisher("/delta_robot/pos_cmd", Vector3, queue_size=10)
    a = Vector3()
    kin = DeltaRobotKinematics()
    z = -800
    t = 0
    while not rospy.is_shutdown():
        try:
            x = 300*sin(t*0.0001)
            y = 300*cos(t*0.0001)
            m1, m2, m3 = kin.reverse(x,y,z)
            a.x, a.y, a.z = radians(m1), radians(m2), radians(m3)
            pub.publish(a)
            t+=1
        except rospy.ROSException or KeyboardInterrupt:
            pass