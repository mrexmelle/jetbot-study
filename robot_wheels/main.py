#!/bin/bash

from jetbot import Robot
import time

robot = Robot()

robot.forward(speed=0.3)
time.sleep(0.5)
robot.left(speed=0.3)
time.sleep(0.5)

robot.stop()
