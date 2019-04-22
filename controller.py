#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Pick(with camera) and Place
"""
import argparse
import struct
import sys
import copy
import random

import rospy
import rospkg

import cv2
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
# custom message
from baxter_interface.camera import CameraController
from gobang.msg import ChessBoard

# 四元数乘法，绕z轴旋转
def rotate_z(ori,angle):
    # qr = [cos(a/2),sin(a/2)*[0,0,1]]
    #[w1 v2][w1 v2] = [wqw2-v1v2,w1v2+w2v1+v2*v1]
    # 原理 https://www.cnblogs.com/mengdd/p/3238223.html
    q = [ori.w,ori.x,ori.y,ori.z]
    ca = math.cos(angle)
    sa = math.sin(angle)
    rs = Quaternion(w = q[0]*ca-q[3]*sa, 
                    x = q[1]*ca-q[2]*sa, 
                    y = q[2]*ca+q[1]*sa, 
                    z = q[3]*ca+q[0]*sa)


# 参考：https://blog.csdn.net/hey_chaoxia/article/details/81914729 
class image_converter:
    def __init__(self,name):
        self._camcon = baxter_interface.CameraController(name+'_hand_camera')
        self._resolution = (480,300) #可选的分辨率 1280x800,960x600,640x400,480x300,384x240
        self._image_sub = rospy.Subscriber('/cameras/'+name+'_hand_camera/image', Image, self._image_callback)
        self._original_image = None #从相机获取的原始图像(opencv格式) 
        self._MAX_SCALE = 100       # 找不到棋子就返回更大的值
        self._pick_bias = (0.025,0.03) # the gripper is not at the center of the camera
        # 颜色在HSV空间中的上下阈值 蓝色：[90,130,30],[130,200,150]
        self._color_dict = {'blue':[[90,50,40],[130,220,100]]} 
    def open_cam(self,on = True):
        #打开相机
        if on:
            #TODO 考虑把头上的关掉，可能有占用的问题 Baxter只允许同时开两个
            self._camcon.resolution = self._resolution
            self._camcon.open()
        else:
            self._camcon.close()

    #转换为opencv图像并更新
    def _image_callback(self, img_data):
        try:
            self._original_image = CvBridge().imgmsg_to_cv2(img_data, "bgr8") #从ros image转换到openCV的图像格式
        except CvBridgeError as e:
            print e

    # 图像处理
    # 对图像的初步处理步骤如下：
    # 转换到HSV图像空间（HSV空间更容易分辨颜色）-->
    # 提取图像中的蓝色部分-->
    # 腐蚀与膨胀去除噪点-->转换为灰度图像-->二值化
    def _image_process(self, color='blue'): 
        # Convert BGR to HSV
        original = self._original_image.copy()
        hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)    #转换到HSV颜色空间 
        lower_color = np.array(self._color_dict[color][0])
        upper_color = np.array(self._color_dict[color][1])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_color, upper_color)   #在hsv颜色空间中获取图像中的颜色部分，用inRange制作掩膜，即该部分为感兴趣的部分 
        res = cv2.bitwise_and(original, original, mask= mask)  #原图像与掩膜进行与操作，则只剩下掩膜部分
        #开操作
        # kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)) #
        # open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)) #获得构造元素
        open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
        # 二值化 
        gray_img = cv2.cvtColor(open_img, cv2.COLOR_BGR2GRAY)
        ret, bin_img = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours)<1: # 没有找到物体
            return [self._MAX_SCALE,self._MAX_SCALE,200]
        [bx,by,bw,bh,ba]=[0]*5
        best = 1000
        scale = 0.0007 # per pixel #TODO by resolution and height
        for c in contours:
            # find bounding box coordinates
            # x, y, w, h = cv2.boundingRect(c)            
            rect = cv2.minAreaRect(bin_img)            
            x, y = rect[0]   # 中心坐标
            w, h = rect[1]]  # 长宽,总有 width>=height
            angle = rect[2]  # 角度:[-90,0)            
            now = abs(x-self._resolution[0]-self._pick_bias[0]/scale)+abs(y-self._resolution[1]+self._pick_bias[1]/scale)
            if now<best:
                [bx,by,bw,bh,ba,best]= [x,y,w,h,angle,now]
        original = original.clone()
        box = cv2.boxPoints(rect)   
        box =np.int0(box)#int32 / int64
        cv2.drawContours(original, [box], 0, (0, 0, 255), 3)  # 画出该矩形
        # cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)
        print bx,by,bw,bh,ba
        cv2.imshow("Searched", original)
        cv2.waitKey(500) #TODO 减少显示时间提高速度
        return [(bx-self._resolution[0]/2)*scale-self._pick_bias[0],-(by-self._resolution[1]/2)*scale-self._pick_bias[1],abs(max(ba,-90-ba))]

# 棋盘信息
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
                self._limb.move_to_joint_positions(joint_angles,timeout = 1.0)
            else:
                self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    # 默认竖直姿态
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
        self._guarded_move_to_joint_position(joint_angles,up)
    def _servo_to_pose(self, pose,quick=False):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles,quick)
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
        self.gripper_open()
        self.statepub.publish(3)   
        self.move_to_wait()   
        self.statepub.publish(4)  
        print 'searching chess ...'
        # wait power on
        rospy.sleep(8)
        [bias_x,bias_y,bias_angle] = self._image_processor._image_process() #m
        print bias_x,bias_y,bias_angle
        while bias_angle>100:
            #TODO 要是永远找不到……/错误的位置的限制？
            # 随机移动
            current_pose = self._limb.endpoint_pose()
            current_pose['position'].x = current_pose['position'].x + (random.randint(1,5) - 3)*0.02
            current_pose['position'].y = current_pose['position'].y + (random.randint(1,5) - 3)*0.02
            self._servo_to_pose(current_pose,False)
            [bias_x,bias_y,bias_angle] = self._image_processor._image_process()
        print 'find chess!'
        # 细微调整
        while abs(bias_x)>0.003 or abs(bias_y)>0.003 or bias_angle > 5:
            current_pose = self._limb.endpoint_pose()
            current_pose['position'].x = current_pose['position'].x+bias_y
            current_pose['position'].y = current_pose['position'].y-bias_x
            current_pose['orientation'] = rotate_z(current_pose['orientation'],bias_angle)
            self._servo_to_pose(current_pose,True) # need quick shutdown to keep images still
            rospy.sleep(0.2) # wait to move
            [bias_x,bias_y,bias_angle] = self._image_processor._image_process()
            print bias_x,bias_y,bias_angle
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
        # 考虑棋盘的角度
        pose = self._limb.endpoint_pose()
        pose['position'].x = x
        pose['position'].y = y
        pose['position'].z = self._board_cfg._height
        pose['orientation'] = rotate_z(self._quat,self._board_cfg._bt)
        self.__place__(pose)
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
    
    # Starting Joint angles for left arm TODO
    starting_joint_angles = {limb+'_w0': 0.6699952259595108,
                             limb+'_w1': 1.030009435085784,
                             limb+'_w2': -0.4999997247485215,
                             limb+'_e0': -1.189968899785275,
                             limb+'_e1': 1.9400238130755056,
                             limb+'_s0': -0.08000397926829805,
                             limb+'_s1': -0.9999781166910306}
    # 参数们，订阅了话题可以在运行时被修改
    [bx,by,bt,bg,bex,bey,bh]=[0.83,0.09,0,0.04,0.6,-0.55,-0.25]
    pnp = PickAndPlace(limb,bex,bey,bx,by,bt,bg,bh,hover_distance)
    pnp.move_to_start(starting_joint_angles)

    def exiting():
        print 'exiting arm controller...'
    rospy.on_shutdown(exiting)
    
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
