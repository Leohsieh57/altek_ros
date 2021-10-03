#! /usr/bin/env python3
'''
1. transform image from cv_mat to rosmsg format
2. assign to sensor_msgs.msg.Image
3. publish compressed color rosmsg
'''

'''ros utils'''
import rospy
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, CompressedImage
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import csv
import sys
import cv2

def CV2msg(cv_image):
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    return image_message

def msg2CV(msg):
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        return image
    except CvBridgeError as e:
        print(e)
        return

def cbDepth_altek(msg):
    print("receive depth_altek!")
    print("altek s: ",msg.header.stamp.secs)
    print("altek ns: ",msg.header.stamp.nsecs)
    cvimgDepth = msg2CV(msg)

    # cv2.imshow('cbDepth_altek', cvimgDepth)
    # cv2.waitKey(5)


def cbDepth(msg):
    print("receive depth!")
    print("before s: ",msg.header.stamp.secs)
    print("before ns: ",msg.header.stamp.nsecs)
    cvimgDepth = msg2CV(msg)
    cv2.imshow('cbDepth', cvimgDepth)
    cv2.waitKey(5)

    msgDepth = CV2msg(cvimgDepth)
    msgDepth.header.stamp.secs = msg.header.stamp.secs
    msgDepth.header.stamp.nsecs = msg.header.stamp.nsecs
    pubDepth.publish(msgDepth)
    
def cbColor_altek(msg):
    print("receive color_altek!")
    print("altek s: ",msg.header.stamp.secs)
    print("altek ns: ",msg.header.stamp.nsecs)
    # cvimgColor = msg2CV(msg)


def cbColor(msg):
    print("receive color!")
    print("color before s: ",msg.header.stamp.secs)
    print("color before ns: ",msg.header.stamp.nsecs)

    cvimgColor = msg2CV(msg)
    # cv2.imshow('cbColor', cvimgColor)
    # cv2.waitKey(5)

    now = rospy.get_rostime()
    msgColor = CV2msg(cvimgColor)
    msgColor.header.stamp.secs = now.secs
    msgColor.header.stamp.nsecs = now.nsecs
    pubColor.publish(msgColor)
    

if __name__ == "__main__":
    print("Python version: ",sys.version)
    rospy.init_node("cv_2_rosmsg", anonymous=True)
    # subDepth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, cbDepth)
    pubDepth = rospy.Publisher("/Altek/depth/image_rect_raw", Image, queue_size=100)
    subDepth_altek = rospy.Subscriber("/Altek/depth/image_rect_raw", Image, cbDepth_altek)

    # subColor = rospy.Subscriber("/camera/color/image_raw", Image, cbColor)
    pubColor = rospy.Publisher("/Altek/color/image_raw", Image, queue_size=100)
    subColor_altek = rospy.Subscriber("/Altek/color/image_raw/compressed", CompressedImage, cbColor_altek)

    print("successfully initialized!")
    

    rospy.spin()
