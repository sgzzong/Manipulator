#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import env
from math import *
import math
import time

class agent():
    def __init__(self):
        self.ned2 = env.Ned2_control()
        self.angle = [[90,20,30,0,0,0],[30,10,20,20,20,0],[-40,-20,30,-20,-10,0],[-10, 20, -30,10,10,0]]
    def train(self):
        self.ned2.reset()
        r = 0
        for i in range(1000):
            print("================start step================")
            self.ned2.action(self.angle[r])
            print("state : ", self.ned2.state())
            print("reward : ", self.ned2.reward())
            r += 1
            if r == 3:
                r = 0
            time.sleep(0.5)
        

def main():
    rl = agent()
    rl.train()
if __name__ == '__main__':
    main()