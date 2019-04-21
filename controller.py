#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Pick(with camera) and Place
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

import cv2
#import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
    Int16MultiArray,
    Char
)
import std_srvs.srv
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    ListCameras
)

import baxter_interface
from baxter_interface.camera import CameraController
from gobang.msg import ChessBoard

#https://blog.csdn.net/hey_chaoxia/article/details/81914729 
class image_converter:
    def __init__(self,name):
        self._camcon = baxter_interface.CameraController(name+'_hand_camera')
        self._resolution = (480,300) #1280x800,960x600,640x400,480x300,384x240
        self._image_sub = rospy.Subscriber('/cameras/'+name+'_hand_camera/image', Image, self._image_callback)
        self._original_image = None #从相机获取的原始图像(opencv格式) 
        self._bin_img = None    #经过处理后的二值图像 
        self._MAX_SCALE = 100 # 找不到就返回更大的值
        self._pick_bias = (0.025,0.03) # the gripper is not at the center of the camera
        self._bias_x = 0
        self._bias_y = 0
        self._color_dict = {'blue':[[90,50,40],[130,220,100]]} #颜色在HSV空间中的上下阈值 [90,130,30],[130,200,150]
    def open_cam(self,on = True):
        #打开相机
        if on:
            self._camcon.resolution = self._resolution
            self._camcon.open()
        else:
            self._camcon.close()

    #转换为opencv图像并更新
    def _image_callback(self, img_data):
        try:
            self._original_image = CvBridge().imgmsg_to_cv2(img_data, "bgr8") #从ros image转换到openCV的图像格式
            #print 'get image'
        except CvBridgeError as e:
            print e
    # 对图像的初步处理步骤如下：转换到HSV图像空间（HSV空间更容易分辨颜色）-->提取图像中的蓝色部分-->腐蚀与膨胀去除噪点-->转换为灰度图像-->二值化
    #图像处理
    def _image_process(self, color='blue'): 

        # Convert BGR to HSV
        original = self._original_image.copy()
        hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)    #转换到HSV颜色空间 
        lower_color = np.array(self._color_dict[color][0])
        upper_color = np.array(self._color_dict[color][1])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_color, upper_color)   #在hsv颜色空间中获取图像中的颜色部分，用inRange制作掩膜，即该部分为感兴趣的部分 
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(original, original, mask= mask)  #原图像与掩膜进行与操作，则只剩下掩膜部分
        #腐蚀操作
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)) #获得构造元素
        open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
 
        gray_img = cv2.cvtColor(open_img, cv2.COLOR_BGR2GRAY)
        ret, self._bin_img = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
        #cv2.imshow("原图",self._original_image)
        #cv2.imshow("二值化",self._bin_img)

        #TODO 计算偏移 还需验证……
        # rect = cv2.minAreaRect(self._bin_img)
        # box = cv2.cv.BoxPoints(rect)
        # box = np.int0(box)
        # cv2.drawContours(img_show, [box], 0, (0, 0, 255), 2)
        contours, hierarchy = cv2.findContours(self._bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x=None
        [bx,by,bw,bh]=[0]*4
        best = 1000
        for c in contours:
            # find bounding box coordinates
            # 现计算出一个简单的边界框
            x, y, w, h = cv2.boundingRect(c)
            #TODO 找一个最近的
            now = abs(x-self._resolution[0])+abs(y-self._resolution[1])
            if now<best:
                [bx,by,bw,bh,best]= [x,y,w,h,now]
                # bx = x
                # by = y
                # bw = w
                # bh = h
                #best = now
            # 画出矩形
            cv2.rectangle(original, (x,y), (x+w, y+h), (0, 255, 0), 2)
        # original = original.clone()
        # cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print bx,by,bw,bh
        cv2.imshow("Searched", original)
        cv2.waitKey(1000)
        scale = 0.0007 # per pixel
        if x:
            return [(bx+bw/2-self._resolution[0]/2)*scale-self._pick_bias[0],-(by+bh/2-self._resolution[1]/2)*scale-self._pick_bias[1]]
        else:
            sys.exit()
            #return [self._MAX_SCALE+1,self._MAX_SCALE+1]
        #TODO scale
        #return (rect.center-cvPoint2D32f(hsv.shape[1],hsv.shape[2]))*scale

class chess_board(object):
    def __init__(self,bx,by,bt,bg,beginx,beginy,height):
        self._bx = bx
        self._by = by
        self._bt = bt # theta 
        self._bg = bg # gap of grid
        self._begin_x = beginx
        self._begin_y = beginy
        self._height = height
        rospy.Subscriber('Info/Board',ChessBoard,self.cb_chess_board)
    def cb_chess_board(info):
        self.bx = info.x
        self.by = info.y
        self.bt = info.theta
        self.bg = info.gap
        self._begin_x = info.bx
        self._begin_y = info.by
        self._height = info.h
        print info


class PickAndPlace(object):
    def __init__(self, limb,begin_x,begin_y,board_x,board_y,board_t,board_g,board_height ,hover_distance = 0.15, verbose=False):
        
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._quat = Quaternion(x=0,y=1,z=0,w=0)
        self._strat_angle = [0]*7 #TODO
        self._board_cfg = chess_board(board_x,board_y,board_t,board_g,begin_x,begin_y,board_height)
        self._image_processor = image_converter(limb)        
        self.statepub = rospy.Publisher('ai_state',Char,queue_size=10)
        
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        
    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, self._strat_angle))
        self._guarded_move_to_joint_position(start_angles,False)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")
    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints
    def _guarded_move_to_joint_position(self, joint_angles,fast):
        if joint_angles:
            if fast:
                self._limb.move_to_joint_positions(joint_angles,timeout = 1.8)
            else:
                self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    def posi2pose(self,x,y,z):
        pose = Pose()
        pose.orientation = self._quat
        pose.position = Point(x,y,z)
        return pose
    def move_to_wait(self):        
        self._approach(self.posi2pose(self._board_cfg._begin_x,self._board_cfg._begin_y,self._board_cfg._height))
    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0) #TODO 夹子不会很快就被操作，可以考虑去掉这个睡眠提高速度
    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)
    def _approach(self, pose,fast=False):
        # approach with a pose the hover-distance above the requested pose
        approach = copy.deepcopy(pose)
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles,fast)
    def _vertical(self,up=True):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        if up:
            ik_pose.position.z = current_pose['position'].z + self._hover_distance
        else:
            ik_pose.position.z = current_pose['position'].z - self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles,False)
    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles,False)
    def _pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._vertical()
    def pick(self):       
        self._image_processor.open_cam()
        self.statepub.publish(3)   
        self.move_to_wait()   
        self.statepub.publish(4)  

        print 'searching chess ...'
        #TODO searching   
        rospy.sleep(8)
        [bias_x,bias_y] = self._image_processor._image_process() #m
        rospy.sleep(2)
        print bias_x,bias_y
        while abs(bias_x)>self._image_processor._MAX_SCALE:
            #TODO can't find
            pass
        # In Scale!
        while abs(bias_x)>0.003 or abs(bias_y)>0.003:
            current_pose = self._limb.endpoint_pose()
            _x = current_pose['position'].x
            _y = current_pose['position'].y
            self._approach(self.posi2pose(_x+bias_y,_y-bias_x,self._board_cfg._height),True) # need quick shutdown to keep images still
            rospy.sleep(0.2) # wait to move
            [bias_x,bias_y] = self._image_processor._image_process()
            print bias_x,bias_y
            if rospy.is_shutdown():
                return 0
        #self._image_processor.open_cam(False)

        self.statepub.publish(5)  
        print 'picking ...'
        self._vertical(False)
        self.gripper_close()
        self._vertical()
    def __place__(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self.statepub.publish(7)
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._vertical()
    def place(self, row,col):
        # 棋盘坐标
        x = self._board_cfg._bx + (col*math.sin(self._board_cfg._bt)-row*math.cos(self._board_cfg._bt))*self._board_cfg._bg
        y = self._board_cfg._by - (col*math.cos(self._board_cfg._bt)+row*math.sin(self._board_cfg._bt))*self._board_cfg._bg
        print row,col,x,y    
        self.statepub.publish(6)    
        print 'placing the chess...'
        #TODO 考虑棋子的角度
        self.__place__(self.posi2pose(x,y,self._board_cfg._height))  
        self.statepub.publish(8) 
        print 'moving to wait...'   
        self.move_to_wait()  
        self.statepub.publish(1)  
        print 'done'  


def cb_move(cor):
    global ai_row,ai_col,begin
    co = cor.data
    ai_row = co[0]
    ai_col = co[1]
    begin = True

def main():
    """
    """    
    #TODO 设置参数选limb
    limb = 'right'
    hover_distance = 0.15 # meters

    rospy.init_node("arm_controller")
    rospy.Subscriber('posi/ai',Int16MultiArray,cb_move)
    
    # Starting Joint angles for left arm
    starting_joint_angles = {limb+'_w0': 0.6699952259595108,
                             limb+'_w1': 1.030009435085784,
                             limb+'_w2': -0.4999997247485215,
                             limb+'_e0': -1.189968899785275,
                             limb+'_e1': 1.9400238130755056,
                             limb+'_s0': -0.08000397926829805,
                             limb+'_s1': -0.9999781166910306}
    [bx,by,bt,bg,bex,bey,bh]=[0.83,0.09,0,0.04,0.6,-0.55,-0.25]
    pnp = PickAndPlace(limb,bex,bey,bx,by,bt,bg,bh,hover_distance)
    pnp.move_to_start(starting_joint_angles)
    global begin,ai_col,ai_row
    begin = False
    ai_row = 7
    ai_col = 7
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if begin:
            begin = False
            print 'begin to grip'
            pnp.pick()
            print 'begin to move to',ai_row,ai_col
            pnp.place(ai_row,ai_col)
    return 0

if __name__ == '__main__':
    sys.exit(main())
