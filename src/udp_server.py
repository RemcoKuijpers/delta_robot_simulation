#!/usr/bin/env python

import socket
import re
from delta_robot import DeltaRobot
import time
import rospy


class UDPClientRobots(object):
    def __init__(self):
        self.robot = DeltaRobot("delta_robot1")
        self.robot2 = DeltaRobot("delta_robot2")
        TCP_IP = rospy.get_param("/robot_controller_ip_address")
        TCP_PORT = 5005
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((TCP_IP, TCP_PORT))
        print("UDP connection for robots started")

    def listen(self):
        while True:
            try:
                data = self.s.recvfrom(81)
                if not data: break
                angles = [twos_comp(int(s), 16) for s in re.findall(r'-?\d+', data[0])]
                self.robot.moveMotors(angles[0], angles[1], angles[2], angles[6])
                self.robot2.moveMotors(angles[3], angles[4], angles[5], angles[7])
            except KeyboardInterrupt:
                break
            except UnicodeDecodeError:
                pass

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

if __name__ == "__main__":
    robots = UDPClientRobots()
    robots.listen()