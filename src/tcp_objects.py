#!/usr/bin/env python

import socket
import time
import re
from object_spawner import PotatoSpawner

class TCPServerObjectSpawner(object):
    def __init__(self):
        TCP_IP = '10.139.10.6'
        TCP_PORT = 5006
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)
        print("Waiting for client to connect...")
        self.conn, addr = s.accept()
        print("Client connected: ", addr)
        self.listen()

    def listen(self):
        self.p = PotatoSpawner()
        i = 1
        while True:
            BUFFER_SIZE = 1000
            data = self.conn.recv(BUFFER_SIZE)
            if not data: break
            data = float(re.findall(r'\d+', data.decode())[0])/1000
            self.p.spawnPotato(data, i)
            i += 1

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val     

if __name__ == "__main__":
    obj = TCPServerObjectSpawner()
    try:
        obj.listen()
    except KeyboardInterrupt:
        for i in range(1,1501):
            obj.p.deletePotato(i)