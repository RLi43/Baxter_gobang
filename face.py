#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import argparse

import rospy

import cv2
import cv_bridge
import numpy as np
import random
import time

from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import Char

# TODO Control Navigator Lights http://sdk.rethinkrobotics.com/wiki/API_Reference#
# TODO Sonor to show face http://sdk.rethinkrobotics.com/wiki/API_Reference#Sonar
# IR Range




# Progress: 1: Waiting 2: Thingking 3: Moving 4: Seaching 5: Picking 6: Moving 7: Placing 8: Moving 11: Pause 12: AI-Win 13: Human-Win


# def send_image(path):
#     """
#     Send the image located at the specified path to the head
#     display on Baxter.

#     @param path: path to the image file to load and send
#     """
#     img = cv2.imread(path)
#     msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
#     pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
#     pub.publish(msg)
#     # Sleep to allow for image to be published.
#     rospy.sleep(1)

class face(object):
    def __init__(self):
        self.state = 1
        self.imgs = {
            'open_eyes':self.load('open'),
            'close_eyes':self.load('close'),
            'think':self.load('think'),
            'get':self.load('get'),
            'win':self.load('win'),
            'lose':self.load('lose'),
            'normal':self.load('normal'),
            'right':self.load('right'),
            'left':self.load('left'),
            'down_left':self.load('down_left'),
            'down_right':self.load('down_right'),
            'down_mid':self.load('down_mid'),
            'up':self.load('up')
        }
        rospy.init_node('baxter_emotion', anonymous=True)
        rospy.Subscriber('ai_state',Char,self.cb_face)
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    def load(self,name):
        return cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('img/'+name+'.png'), encoding="bgr8")
    def cb_face(self,ai_s):
        self.state = ai_s      
    def show_image(self,name):
        self.pub.publish(self.imgs[name])
        #rospy.sleep(1) #
    def go(self):
        self.show_image('normal')
        old_state = self.state
        winkle_delay = 3+random.randint(0,100)/50.0
        time_last = time.time()
        rate = rospy.Rate(20)
        open_eye = True
        open_pos = 'normal'
        while not rospy.on_shutdown():
            if old_state == self.state:
                # waiting for human
                if (self.state == 1 or self.state == 11 or self.state == 2):
                    #眨眼 
                    #TODO 做眼神什么的 感觉更智能了呢
                    if open_eye:
                        if time.time()-time_last >= winkle_delay:
                            time_last = time.time()
                            self.show_image('close_eyes')
                            winkle_delay = 3+random.randint(0,100)/50.0
                            open_eye = False
                    else:
                        if time.time()-time_last >= 1.0:
                            time_last = time.time()
                            self.show_image(open_pos)
                            open_eye = True
                    if (self.state != 2): #Not Thinking
                        temp_rand = random.randint(0,20)
                        if temp_rand>17:
                            open_pos = 'right'
                        elif temp_rand<3:
                            open_pos = 'left'
                        elif temp_rand < 6:
                            open_pos = 'down_mid'
                        else:
                            open_pos = 'normal'
                    else:
                        open_pos = 'think'
            else:
                old_state = self.state
                if self.state == 3:
                    self.show_image('get')
                elif self.state == 4:
                    self.show_image('down_left')
                elif self.state == 6 or self.state == 5 or self.state == 7:
                    self.show_image('down_mid')
                elif self.state == 1:
                    self.show_image('normal')
                elif self.state == 2:
                    self.show_image('think')
                elif self.state == 12:
                    self.show_image('win')
                elif self.state == 13:
                    self.show_image('lose')
                else:
                    self.show_image('normal')
            rate.sleep()
    # if not os.access(args.file, os.R_OK):
    #     rospy.logerr("Cannot read file at '%s'" % (args.file,))
    #     return 1
    #         if args.delay > 0:
    #     rospy.loginfo(
    #         "Waiting for %s second(s) before publishing image to face" %
    #         (args.delay,)
    #     )
    #     rospy.sleep(args.delay)
    # send_image(args.file)


if __name__ == '__main__':
    fa = face()
    sys.exit(fa.go())
