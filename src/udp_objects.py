#!/usr/bin/env python

import socket
import re
import time
import rospy
from object_spawner import PotatoSpawner

class UDPClientObjectSpawner():
    def __init__(self):
        UDP_IP = rospy.get_param("/robot_controller_ip_address")
        potatoes = rospy.get_param("/amount_of_potatoes")
        UDP_PORT = 5006
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((UDP_IP, UDP_PORT))
        self.p = PotatoSpawner()
        self.p.spawnManyPotatos(potatoes)
        print("UDP connection for objects started")

    def listen(self):
        while True:
            try:
                data = self.s.recvfrom(81)
                data = re.findall(r'-?\d+', data[0])
                if data != []:
                        num = int(data[0])
                        x = float(data[1])/1000
                        y = float(data[2])/1000
                        rz = -float(data[3])+90
                        self.p.updatePoseNew(num,x,y,rz,25)
            except KeyboardInterrupt:
                break
            except IndexError:
                pass
                    
if __name__ == "__main__":
    obj = UDPClientObjectSpawner()
    obj.listen()