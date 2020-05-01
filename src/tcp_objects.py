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
        #s.listen(1)
        print("Waiting for client to connect...")
        #self.conn, addr = s.accept()
        #print("Client connected: ", addr)
        self.p = PotatoSpawner()
        self.p.spawnManyPotatos(30)

    def listen(self):
        data = self.s.recvfrom(81)
        data = re.findall(r'-?\d+', data[0])
        if data != []:
            try:
                num = int(data[0])
                x = float(data[1])/1000
                y = float(data[2])/1000
                self.p.updatePose(num,x,y)
            except IndexError:
                pass
                
if __name__ == "__main__":
    obj = TCPServerObjectSpawner()
    while True:
        obj.listen()