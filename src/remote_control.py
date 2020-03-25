#!/usr/bin/env python

import pygame
from delta_robot import DeltaRobot

def main():
    robot = DeltaRobot()
    pygame.init()
    pygame.display.set_mode((640, 480))
    pygame.time.Clock()
    x = 0
    y = 0
    z = -700
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    z += 10
                elif event.button == 5:
                    z -= 10
            if event.type == pygame.MOUSEMOTION:
                x = (event.pos[0] - 320) * 0.625
                y = (event.pos[1] - 240) * 0.8333333333333
                print(x,y)

        robot.movePositionCartesian(x, y, z)
main()