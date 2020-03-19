#!/usr/bin/env python

import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

class DeltaPositionError(Exception):
    pass

class DeltaRobotKinematics(object):

    def __init__(self):
        self.e = 75
        self.f = 200
        self.re = 850
        self.rf = 320

    def forward(self, theta1, theta2, theta3):
        """ 
        Takes three servo angles in degrees.  Zero is horizontal.
        return (x,y,z) if point valid, None if not 
        """
        t = self.f-self.e

        theta1, theta2, theta3 = math.radians(theta1), math.radians(theta2), math.radians(theta3)

        # Calculate position of leg1's joint.  x1 is implicitly zero - along the axis
        y1 = -(t + self.rf*math.cos(theta1))
        z1 = -self.rf*math.sin(theta1)

        # Calculate leg2's joint position
        y2 = (t + self.rf*math.cos(theta2))*math.sin(math.pi/6)
        x2 = y2*math.tan(math.pi/3)
        z2 = -self.rf*math.sin(theta2)

        # Calculate leg3's joint position
        y3 = (t + self.rf*math.cos(theta3))*math.sin(math.pi/6)
        x3 = -y3*math.tan(math.pi/3)
        z3 = -self.rf*math.sin(theta3)

        # From the three positions in space, determine if there is a valid
        # location for the effector
        dnm = (y2-y1)*x3-(y3-y1)*x2
    
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3

        # x = (a1*z + b1)/dnm
        a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
        b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0

        # y = (a2*z + b2)/dnm;
        a2 = -(z2-z1)*x3+(z3-z1)*x2
        b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0

        # a*z^2 + b*z + c = 0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
        c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - self.re*self.re)
 
        # discriminant
        d = b*b - 4.0*a*c
        if d < 0:
            return None # non-existing point

        z0 = -0.5*(b+math.sqrt(d))/a
        x0 = (a1*z0 + b1)/dnm
        y0 = (a2*z0 + b2)/dnm
        return (x0,y0,z0)


    def _calcAngleYZ(self, x0, y0, z0):
        y1 = -self.f
        y0 -= self.e
        a = (x0*x0 + y0*y0 + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1)/(2*z0)
        b = (y1-y0)/z0
        d = -(a + b*y1)*(a + b*y1) + self.rf*(b*b*self.rf + self.rf)
        if d < 0:
            raise DeltaPositionError()
        yj = (y1 - a*b - math.sqrt(d))/(b*b + 1)
        zj = a + b*yj
        theta = 180.0*math.atan(-zj/(y1-yj))/math.pi
        if yj>y1:
            theta += 180.0
        return theta


    def reverse(self, x0, y0, z0):
        """
        Takes position and returns three servo angles, or 0,0,0 if not possible
        return (x,y,z) if point valid, None if not
        """
        cos120 = math.cos(2.0*math.pi/3.0)
        sin120 = math.sin(2.0*math.pi/3.0)

        try:
            theta1 = self._calcAngleYZ(x0, y0, z0)
            theta2 = self._calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120 - x0*sin120, z0) # rotate +120 deg
            theta3 = self._calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120 + x0*sin120, z0) # rotate -120 deg

            return theta1, theta2, theta3
        except DeltaPositionError:
            return 0,0,0

    def plotWorkingRange(self):
        minServo = -47
        maxServo = 99
        step = 30
        points = []
        for t1 in range(minServo, maxServo, step):
            for t2 in range(minServo, maxServo, step):
                for t3 in range(minServo, maxServo, step):
                    servos = (t1, t2, t3)
                    points.append(bot.forward(*servos))
                    there_and_back = bot.reverse(*bot.forward(*servos))
                    err = map(lambda a,b: abs(a-b), servos, there_and_back)
                    if max(err) > 0.0000000000001:
                        print(servos, there_and_back, err)

        fig = plt.figure()
        ax = fig.add_subplot(1,1,1, projection='3d')
        surf = ax.scatter(xs=[x for x,y,z in points] ,ys=[y for x,y,z in points],zs=[z for x,y,z in points])
        plt.show()



if __name__ == '__main__':
    bot = DeltaRobotKinematics()
    bot.plotWorkingRange()