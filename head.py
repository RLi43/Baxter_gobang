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
    def __init__(self):
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
        self.sonar = None
        self.state = 1
        rospy.Subscriber('/robot/sonar/head_sonar/state',PointCloud,self.cb_sonar)
        rospy.Subscriber('ai_state',Char,self.cb_state)
    def pan(self,angle):
        self.head.set_pan(angle,0.5,2) #speed , timeout   
    def go(self):
        old_state = self.state
        i = 0
        a = 0.7
        old = 0
        while not rospy.is_shutdown():
            if old_state == self.state:
                # waiting, head follow
                if self.state == 1 or self.state >= 11:
                    #TODO head follow
                    if self.sonar:
                        #print self.sonar
                        get_id = self.sonar[0].values
                        value = self.sonar[1].values
                        if get_id:
                            for i in range(len(get_id)):
                                if i>6: i = i-12
                            va = []
                            fa = False
                            for i in range(len(value)):
                                if value[i]<1.2:
                                    va.append(0)
                                else:
                                    va.append( 5 - value[i])
                                    fa = True
                            # va = [5 - value[i] for i in range(len(value))]
                            # index = get_id[value.index(min(value))]
                            if fa:
                                print self.sonar
                                index = np.average(get_id,weights = va)
                                print 'index', index
                                if index > 6:
                                    index = -(index-12)/4
                                else: index = -index/4
                                index = index*a + old*(1-a)
                                if index>1.3: index = 1.3
                                elif index<-1.3:index = -1.3
                                old = index
                                self.pan(index)
                                rospy.sleep(2)
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
        self.sonar = data.channels


if __name__ == '__main__':
    head = Head()
    sys.exit(head.go())

        
