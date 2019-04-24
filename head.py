#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import argparse

import rospy

import numpy as np
import random
import time
import baxter_interface
from std_msgs.msg import Char,Float32,UInt16
from sensor_msgs.msg import PointCloud

# TODO Control Navigator Lights http://sdk.rethinkrobotics.com/wiki/API_Reference#
# TODO Sonor to show face http://sdk.rethinkrobotics.com/wiki/API_Reference#Sonar
# IR Range




# Progress: 1: Waiting 2: Thingking 3: Moving 4: Seaching 5: Picking 6: Moving 7: Placing 8: Moving 11: Pause 12: AI-Win 13: Human-Win

class haloLED(object):
    def __init__(self)
        self.red = rospy.Publisher("/robot/sonar/lights/set_red_level",Float32,queue_size = 2)
        self.green = rospy.Publisher("/robot/sonar/lights/set_green_level",Float32,queue_size = 2)
        # read_red = rospy.Subsriber
    def set_red(self,value):
        self.red.publish(value)
    def set_green(self,value):
        self.green.publish(value)
    def set_both(red,green):
        self.set_red(red)
        self.set_green(green)

class Head(object):
    def __init__(self):
        rospy.init_node('head_controller')
        self.head = baxter_interface.Head()
        self.led = haloLED()
        self.sonarLED = 0
        self.state = 0
        rospy.Subsriber('/robot/sonar/head_sonar/lights/state',UInt16,self.cb_sonar)
        rospy.Subsriber('ai_state',Char,self.cb_state)
    def pan(self,angle):
        self.head.set_pan(angle,100,2) #speed , timeout   
    def go(self):
        old_state = self.state
        i = 0
        while not rospy.is_shutdown():
            if old_state == self.state:
                # waiting, head follow
                if self.state == 1 or self.stae >= 11:
                    #TODO head follow
                    
                    # win or lose
                    if self.state == 12 or self.state == 13:
                        i = (i+1)%101
                        self.led.set_both(100-i,i)
                    else :
                        i = 0
            else:
                old_state == self.state
                if self.state == 12 or self.state == 13:
                    self.led.set_bot(100,0)

    def cb_state(self,data):
        self.state = data.data

    def cb_sonar(self,data):
        self.sonarLED = data.data


if __name__ == '__main__':
    head = Head()
    sys.exit(head.go())

        
