#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg

import cv2
#import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import baxter_interface
from baxter_interface.camera import CameraController

class img_reader(object):
    def __init__(self,name):
        self._camcon = baxter_interface.CameraController(name)
        self._original_image = None
        self._image_sub = rospy.Subscriber('/cameras/'+name+'/image', Image, self._image_callback)
    def _image_callback(self, img_data):
        print 'get image'
        try:
            self._original_image = CvBridge().imgmsg_to_cv2(img_data, "bgr8") #从ros image转换到openCV的图像格式
        except CvBridgeError as e:
            print e
    def save(self):
        cv2.imwrite('now.png',self._original_image)
    def show(self):
        cv2.imshow("test",self._original_image)
        cv2.waitKey(0)
    def open_cam(self,on = True):
        #打开相机
        if on:
            self._camcon.resolution = (1280,800)
            self._camcon.open()
        else:
            self._camcon.close()


if __name__ == '__main__':
    rospy.init_node('image_reader')
    reader = img_reader('right_hand_camera')#right_hand_camera head_camera
    reader.open_cam()
    rospy.sleep(5)
    reader.show()
    #reader.open_cam(False)
