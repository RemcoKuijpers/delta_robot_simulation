#!/usr/bin/env python

import socket
import re
import time
from object_spawner import PotatoSpawner

class TCPServerObjectSpawner():
    def __init__(self):
        TCP_IP = '10.139.10.6'
        TCP_PORT = 5006
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((TCP_IP, TCP_PORT))
        self.p = PotatoSpawner()
        self.p.spawnManyPotatos(25)
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
                        self.p.updatePoseNew(num,x,y,25)
            except KeyboardInterrupt:
                break
            except IndexError:
                pass
                    
if __name__ == "__main__":
    obj = TCPServerObjectSpawner()
    obj.listen()