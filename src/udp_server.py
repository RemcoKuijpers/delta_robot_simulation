#!/usr/bin/env python

import socket
import re
from delta_robot import DeltaRobot
import rospy


class UDPClientRobots(object):
    def __init__(self):
        self.robot = DeltaRobot("delta_robot1", 90)
        self.robot2 = DeltaRobot("delta_robot2", 120+90)
        UDP_IP = rospy.get_param("/robot_controller_ip_address")
        UDP_PORT = 5008
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((UDP_IP, UDP_PORT))

    def listen(self):
        while True:
            try:
                data = self.s.recvfrom(81)
                if not data: break
                angles = [twos_comp(int(s), 16) for s in re.findall(r'-?\d+', data[0])]
                self.robot.moveMotors(float(angles[0])/100, float(angles[1])/100, float(angles[2])/100, -float(angles[6])/100)
                self.robot2.moveMotors(float(angles[3])/100, float(angles[4])/100, float(angles[5])/100, -float(angles[7])/100)
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