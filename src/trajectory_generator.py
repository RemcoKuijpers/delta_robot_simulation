#!/usr/bin/env python

import rospy
import tf
import time
from math import radians
import os, rospkg
import matplotlib.pyplot as plt
import threading
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class TrajectoryGenerator(object):
    def __init__(self):
        rospy.init_node("trajectory_publisher")
        self.broadcaster = tf.TransformBroadcaster()
        rospack = rospkg.RosPack()
        self.dt = []
        self.i = 0
        self.old = 0
        self.t, self.x, self.y, self.z, self.rz = np.loadtxt(os.path.join(rospack.get_path("delta_robot_simulation"), "csv", "Trajectory.csv"), delimiter=",", unpack=True)
        self.getTimeDifference()

    def plotTrajectory2D(self):
        _, axs = plt.subplots(2,2)
        axs[0, 0].plot(self.t, self.x)
        axs[0, 0].set_title('X [mm]')
        axs[0, 1].plot(self.t, self.y)
        axs[0, 1].set_title('Y [mm]')
        axs[1, 0].plot(self.t, self.z)
        axs[1, 0].set_title('Z [mm]')
        axs[1, 1].plot(self.t, self.rz)
        axs[1, 1].set_title('Rz [mm]')
        plt.show()

    def plotTrajectory3D(self, steps):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(self.x[:steps], self.y[:steps], self.z[:steps], label='parametric curve')
        plt.show()

    def getTimeDifference(self):
        for x in range(len(self.t)):
            if x == len(self.t)-1:
                break
            self.dt.append(self.t[x+1] - self.t[x])

    def startTimer(self, us):
        self.timer = threading.Timer(us/1000000, self.sender)
        self.timer.start()

    def sender(self):
        self.broadcaster.sendTransform((self.x[self.i]/1000, self.y[self.i]/1000, self.z[self.i]/1000),
                                        tf.transformations.quaternion_from_euler(0, 0, radians(self.rz[self.i])),
                                        rospy.Time.now(), "trajectory", "world")
        self.timer.cancel()
        self.i += 1
        if self.i == len(self.t)-1:
            return
        self.startTimer(self.dt[self.i])

    def sendTrajectoryToTF(self):
        self.startTimer(self.dt[0])

if __name__ == "__main__":
    traj = TrajectoryGenerator()
    traj.sendTrajectoryToTF()