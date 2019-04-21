import os
import sys
import argparse

import rospy

import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import Char

# Progress: 1: Wating 2: Thingking 3: Moving 4: Seaching 5: Picking 6: Moving 7: Placing 8: Moving 11: Pause 12: AI-Win 13: Human-Win
def cb_face(ai_s):
    global state
    state = ai_s
    
def show_image(state):
    global pub,imgs
    pub.publish(imgs[state])
    rospy.sleep(1)

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

def main():
    global state
    state = 1
    imgs = {
        'open_eyes':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('open.jpg'), encoding="bgr8"),
        'close_eyes':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('close.jpg'), encoding="bgr8"),
        'think':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('think.jpg'),encoding="bgr8"),
        'search':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('search.jpg'),encoding="bgr8"),
        'win':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('win.jpg'),encoding="bgr8"),
        'lose':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('lose.jpg'),encoding="bgr8"),
        'motion':cv_bridge.CvBridge().cv2_to_imgmsg(cv2.imread('motion.jpg'),encoding="bgr8")
    }
    rospy.init_node('baxter_emotion', anonymous=True)
    rospy.Subscriber('ai_state',Char,cb_face)
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
    old_state = state
    import random
    import time
    winkle_delay = 3+random.randint(0,100)/50.0
    time_last = time.time()
    while not rospy.on_shutdown():
        if old_state == state:
            if state == 1 or state == 12:
                #眨眼 
                #TODO 做眼神什么的 感觉更智能了呢
                if time.time()-time_last >= winkle_delay:
                    pub.publish(imgs['close_eyes'])
                    rospy.sleep(1)
                    pub.publish(imgs['open_eyes'])
                    winkle_delay = 3+random.randint(0,100)/50.0
                    time_last = time.time()
            continue
        else:
            old_state = state
            if state == 1 or state == 12
                pub.publish(imgs['wait'])
            elif state == 2:
                pub.publish(imgs['think'])
            elif state == 4:
                pub.publish(imgs['search'])
            elif state == 12:
                pub.publish(imgs['win'])
            elif state == 13:
                pub.publish(imgs['lose'])
            elif state<10 and state>2:
                pub.publish(imgs['motion'])
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

    return 0

if __name__ == '__main__':
    sys.exit(main())