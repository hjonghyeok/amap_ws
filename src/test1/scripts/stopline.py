#!/usr/bin/env python
# -*- coding : utf-8 -*-

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32
import math


bridge = CvBridge()
cv_image = np.empty(shape=[0])
cv_imagef = np.empty(shape=[0])
# To use OpenCV with ROS

X,Y,W,H = 100,100,300,100 # roi setting start position x,y, width, height


def roi_image(image):
    (x,y),(w,h) = (X,Y),(W,H) 
    roi_img = image[y:y+h, x:x+w]

    return roi_img


def white_filter(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    white_lower = np.array([0, 0, 200]) 
    white_upper = np.array([100, 100, 255])

    white_mask = cv2.inRange(hsv, white_lower, white_upper)

    white_masked = cv2.bitwise_and(image, image, mask=white_mask)

    return white_masked


def line_write(stop_lines):   # line detection
    if stop_lines is not None:
        for i in range(0, len(stop_lines)):
            rho = stop_lines[0][0][0]
            theta = stop_lines[0][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a))) 
            cv2.line(white_cdst, pt1, pt2, (30, 225, 225), 2, cv2.LINE_AA)
            pub.publish(1)
    else:
        pub.publish(0)




def img_callback(data):
    global cv_image, cv_imagef
    cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
    cv_imagef = cv_image.copy()
    # cv_imagef = cv2.flip(cv_image, -1)
    # imgmsg data -> opencv
    # "bgr8": CV_8UC3, color image with blue-green-red color order

rospy.init_node('stopline_node', anonymous=True)
rospy.Subscriber("usb_cam4/image_raw",Image,img_callback)
pub = rospy.Publisher("stopline", Int32, queue_size = 10)



while not rospy.is_shutdown():
    if (cv_imagef.size != (640*480*3)): continue
    # wait until image is received. resolution: 640 x 380, true color image
    src = roi_image(cv_imagef)
    
    cv2.rectangle(src, (X, Y), (X+W, Y+H), (255,0,0), 3)
    white_src = white_filter(src)

    white_dst = cv2.Canny(white_src, 300, 600, None, 3)

    white_cdst = cv2.cvtColor(white_dst, cv2.COLOR_GRAY2BGR) 
    white_cdstP = np.copy(white_cdst)

    stop_lines = cv2.HoughLines(white_dst, 1, 5*np.pi / 180, 250, None, 0, 0, 82, 98)

    line_write(stop_lines)
    # print(cv_imagef.shape)

    
    # gray = cv2.cvtColor(cv_imagef,cv2.COLOR_BGR2GRAY)
    # blur_gray = cv2.GaussianBlur(gray,(5,5),0)
    # edge_img = cv2.Canny(blur_gray, 50, 150)

    cv2.imshow("Stop Lines (in red) - Standard Hough Line Transform", white_cdst)
    cv2.imshow("original",cv_imagef)
    # cv2.imshow("gray",gray)
    # cv2.imshow("GaussianBlur",blur_gray)
    # cv2.imshow("edge_img",edge_img)
    cv2.waitKey(1)