#!/usr/bin/env python3

import os
import rospy
from altek_ros.srv import *
from datetime import datetime
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, CompressedImage

import cv2
import numpy as np
import time
import threading
from queue import Queue
# 20210922 #
# support task run
# 20210917 #
# support image preview
#
gMainVer = "[py - 20210922]"

gDistInMin = 200
gDistInMax = 5000
dev = rospy.get_param('altek/video_port') #202: default, 1: two more camera and manual set to 1.
cnt = 10
g_SaveDist = 0
g_wait_ms = 1
g_wait_1s = 1000
DepthLUT_table = [0]*1024
ColorLUT_table = [0]*1024*3
g_mode=2 # 1: depth preview only, 2: depth + image preview.
g_IsStop=0

g_w_dist = 640.0
g_h_dist = 360.0
g_h_nv21 = 540.0
g_h_nv21_dist = 630.0

fnIndex = 0

numpy_type_to_cvtype = {
    'uint8': '8U', 'int8': '8S', 'uint16': '16U',
    'int16': '16S', 'int32': '32S', 'float32': '32F',
    'float64': '64F'
}

numpy_type_to_cvtype.update(dict((v, k) for (k, v) in numpy_type_to_cvtype.items()))

lut_path = rospy.get_param('altek/path/find_lut')
img_path = rospy.get_param('altek/path/save_img')
save_data = rospy.get_param('altek/option/save_data')

if not os.path.exists(img_path) and save_data:
    os.mkdir(img_path)

img_path = os.path.join(img_path, datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
if save_data:
    os.mkdir(img_path)

path_dict = {
    path_key: os.path.join(img_path, path_key) 
        for path_key in (
            'color_img', 'depth_img', 'color_raw', 
            'depth_raw', 'color_npy', 'depth_npy'
        )
}

wait_for_publish_cmd = True

if save_data:
    for key, value in path_dict.items():
        os.makedirs(value)

def CV2msg(cv_image):
    image_message = cv2_to_imgmsg(cv_image, encoding="passthrough")
    return image_message

def cbDepth_altek(msg):
    print("receive depth_altek!")
    print("altek s: ",msg.header.stamp.secs)
    print("altek ns: ",msg.header.stamp.nsecs)
    
    # cv2.imshow('Distance', cvimgDepth)
    # cv2.waitKey(1)

def cbColor_altek(msg):
    print("receive color_altek!")
    print("altek s: ",msg.header.stamp.secs)
    print("altek ns: ",msg.header.stamp.nsecs)

def ShowVersions():
    print(gMainVer)
    print('cv2 ver=%s' % cv2.__version__)
    print('numpy ver=%s' % np.__version__)

def opencv_cfg(cap, g_mode):
    cap.set(cv2.CAP_PROP_FORMAT, -1.0)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
    cap.set(cv2.CAP_PROP_MODE, 0.0)
    if g_mode == 1:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, g_w_dist)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, g_h_dist)
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, g_w_dist)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, g_h_nv21_dist)

def get_opencv_cfg():
    vcformat = cap.get(cv2.CAP_PROP_FORMAT)
    fourcc = cap.get(cv2.CAP_PROP_FOURCC)
    ConvertRGB = cap.get(cv2.CAP_PROP_CONVERT_RGB)
    Mode = cap.get(cv2.CAP_PROP_MODE)
    W = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    H = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print("Get cv2 property:")
    print("vcformat = %f" % vcformat)
    print("fourcc = %f" % fourcc)
    print("ConvertRGB = %f" % ConvertRGB)
    print("Mode = %f" % Mode)
    print("W = %f, H = %f\n" % (W, H))

def LoadDepthLUT_Table():
    i=0
    fn = os.path.join(lut_path, 'depth_lut.txt')
    rf = open(fn, 'r')
    for line in rf.readlines():
        DepthLUT_table[i] = line
        i = i+1
    rf.close()

def LoadColorLUT_Table():
    i=0
    fn = os.path.join(lut_path, 'color_lut.txt')
    rf = open(fn, 'r')
    for line in rf.readlines():
        S0, S1, S2, S3 = line.split(',', 4)
        ColorLUT_table[i] = int(S0, 16)
        ColorLUT_table[i+1] = int(S1, 16)
        ColorLUT_table[i+2] = int(S2, 16)
        i = i+3
    rf.close()

def PixelCvt_Dist2RGBnp(Inframe, width, height, pRGB, InMin, InMax, mapping):
    depthlut = np.zeros((width,height), np.uint16)
    depthlut = 1023 - mapping[Inframe - InMin]
    ftmp=np.reshape(pRGB, (width,height,3))
    
    ftmp[:,:,0]= ColorLUT_table_np[depthlut[:,:]*3+2] # B
    ftmp[:,:,1]= ColorLUT_table_np[depthlut[:,:]*3+1] # G
    ftmp[:,:,2]= ColorLUT_table_np[depthlut[:,:]*3]   # R
    
    ftmp[Inframe > InMax, 0] = 0xFF # B
    ftmp[Inframe > InMax, 1] = 0 # G
    ftmp[Inframe > InMax, 2] = 0 # R

def PixelCvt_Dist2Depth(Inframe, width, height, InMin, mapping):
    depthlut = np.zeros((width,height), np.uint16)
    depthlut = 1023 - mapping[Inframe - InMin]
    print(depthlut.shape, depthlut.dtype)
    return depthlut.astype(np.uint16)

def CheckCamera(cap, dev, cnt):
    # Check camera open status
    while not cap.isOpened() and not rospy.is_shutdown():
        cv2.waitKey(g_wait_1s)
        cap = cv2.VideoCapture(dev, cv2.CAP_ANY)
        print("Wait for opening camera...")
        if cnt <= 0:
            break
    return cnt

def cv2_to_imgmsg(cvim, encoding = "passthrough"):
        """
        Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::Image message.
        :param cvim:      An OpenCV :cpp:type:`cv::Mat`
        :param encoding:  The encoding of the image data, one of the following strings:
           * ``"passthrough"``
           * one of the standard strings in sensor_msgs/image_encodings.h
        :rtype:           A sensor_msgs.msg.Image message
        :raises CvBridgeError: when the ``cvim`` has a type that is incompatible with ``encoding``
        If encoding is ``"passthrough"``, then the message has the same encoding as the image's OpenCV type.
        Otherwise desired_encoding must be one of the standard image encodings
        This function returns a sensor_msgs::Image message on success, or raises :exc:`cv_bridge.CvBridgeError` on failure.
        """
        if not isinstance(cvim, (np.ndarray, np.generic)):
            raise TypeError('Your input type is not a numpy array')

        img_msg = Image()
        img_msg.height = cvim.shape[0]
        img_msg.width = cvim.shape[1]
        if len(cvim.shape) < 3:
            cv_type = '%sC%d' % (numpy_type_to_cvtype[cvim.dtype.name], 1)
        else:
            cv_type = '%sC%d' % (numpy_type_to_cvtype[cvim.dtype.name], cvim.shape[2])
        if encoding == "passthrough":
            img_msg.encoding = cv_type
        else:
            img_msg.encoding = encoding
            # Verify that the supplied encoding is compatible with the type of the OpenCV image

        if cvim.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.data = cvim.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height

        return img_msg


def th_showimg():
    global cap
    global g_h_dist
    global g_h_nv21
    global g_mode
    global frame_cv
    global gDistInMin, gDistInMax
    global mapping
    global g_SaveDist
    global g_IsStop
    global fnIndex
    
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("th w=%d, h=%d, g_mode=%d" % (w, h, g_mode))
    
    if g_mode == 1:
        while(g_IsStop == 0 and not rospy.is_shutdown()):
            #tic1=time.time()
            #print("cycle-t: ",tic1)
            #tic2=tic1

            # print("ONly preview depth...")

            # get 1 image from camera
            ret, frame_cv = cap.read()
            if ret == 0 :
                print("No video frame")
                ch = cv2.waitKey(g_wait_1s)
                continue
            #tic2=time.time()
        
            imgSize = (w,h)
            frame_np = np.frombuffer(frame_cv, dtype=np.uint16).reshape(imgSize)
            fBGR = np.zeros((h,w,3), np.uint8)
            PixelCvt_Dist2RGBnp(frame_np, w, h, fBGR, gDistInMin, gDistInMax, mapping)
            q_d.put(fBGR)
            
            if(g_SaveDist == 1):
                fcv = open('./Image/frame_dist.raw','wb')
                fcv.write(frame_cv)
                fcv.close()
                print("Write frame_cv.raw")
                g_SaveDist = 0
        
    else:
        while(g_IsStop == 0 and not rospy.is_shutdown()):
            #tic1=time.time()
            #print(tic1)
            #tic2=tic1
        
            # get 1 image from camera
            ret, frame_cv = cap.read()
            if ret == 0 :
                print("No video frame")
                ch = cv2.waitKey(g_wait_1s)
                continue
            #tic2=time.time()
        
            imgSize = (h*2,w,1)
            frame_np = np.frombuffer(frame_cv, dtype=np.uint8).reshape(imgSize)
            h_rgb=int(g_h_nv21)
            ftmp = frame_np[:h_rgb,:w]
            fBGRnv21 = cv2.cvtColor(ftmp,cv2.COLOR_YUV2BGR_NV21)
            
            fdist = frame_np[h_rgb:h*2,:w]
            h_d=int(g_h_dist)
            imgSize = (w,h_d)
            frame_npd = np.frombuffer(fdist, dtype=np.uint16).reshape(imgSize)
            fBGRdt = np.zeros((h_d,w,3), np.uint8)
            PixelCvt_Dist2RGBnp(frame_npd, w, h_d, fBGRdt, gDistInMin, gDistInMax, mapping)
            # merge depth + image.
            # fBGR = np.concatenate((fBGRnv21, fBGRdt), axis=1)
            q_d.put(fBGRdt)
            q_rgb.put(fBGRnv21)

            if save_data:
                fn = str(fnIndex).zfill(6)


                fcv = open(os.path.join(path_dict['color_raw'], fn+'.raw'),'wb')
                fcv.write(ftmp)
                fcv.close()

                fcvd = open(os.path.join(path_dict['depth_raw'], fn+'.raw'),'wb')
                fcvd.write(fdist)
                fcvd.close()

                np.save(os.path.join(path_dict['depth_npy'], fn+'.npy'), fBGRdt)
                np.save(os.path.join(path_dict['color_npy'], fn+'.npy'), fBGRnv21)

                cv2.imwrite(os.path.join(path_dict['depth_img'], fn+'.png'), fBGRdt)
                cv2.imwrite(os.path.join(path_dict['color_img'], fn+'.png'), fBGRnv21)

                print("Write frame_cv.raw")

                fnIndex+=1

def wait_for_cmd(msg):
    global wait_for_publish_cmd
    wait_for_publish_cmd = False
    return wait_for_publish_cmd

# show versions
if __name__ == "__main__":
    ShowVersions()

    rospy.init_node("altek_ros", anonymous=True)
    pubDepth = rospy.Publisher("/Altek/depth/image_rect_raw", Image, queue_size=100)
    pubColor = rospy.Publisher("/Altek/color/image_raw", Image, queue_size=100)

    s = rospy.Service('altek_ros/start_publish', start_publish, wait_for_cmd, buff_size=1)


    # select camera, 0, or 1, or ...
    cap = cv2.VideoCapture(dev, cv2.CAP_ANY)
    ret = CheckCamera(cap, dev, cnt)

    q = Queue()
    q_rgb = Queue()
    q_d = Queue()

    rate = rospy.Rate(5)
    while wait_for_publish_cmd and not rospy.is_shutdown():
        rate.sleep()

    rospy.loginfo("Start publish ROS topic! ")
    if cap.isOpened() and not rospy.is_shutdown():
        cnt = 0
        opencv_cfg(cap, g_mode)

        #LoadDepthLUT_Table()
        LoadColorLUT_Table()
        get_opencv_cfg()
        
        table_size = gDistInMax - gDistInMin + 1
        mappinglist = [0]*65536 #table_size
        for i in range(0, table_size):
            mappinglist[i] = round(i * 1023 / (gDistInMax - gDistInMin))
        # converting list to array
        mapping = np.array(mappinglist)
        ColorLUT_table_np = np.array(ColorLUT_table)
        #tic1 = tic2 = 0
        
        t = threading.Thread(target = th_showimg)
        t.start()
        
        while(not rospy.is_shutdown()):

            if g_mode == 1:

                fDepth = q_d.get()
                # cv2.namedWindow('Distance', 1)
                # cv2.imshow('Distance', fBGR)

                '''cvt to rosmsg'''
                now = rospy.get_rostime()
                msgDepth = CV2msg(fDepth)
                msgDepth.header.stamp.secs = now.secs
                msgDepth.header.stamp.nsecs = now.nsecs
                msgDepth.header.frame_id = 'camera_link'
                pubDepth.publish(msgDepth)
            
            else:
                fDepth = q_d.get()
                fBGR = q_rgb.get()

                '''cvt to rosmsg'''
                now = rospy.get_rostime()
                msgDepth = CV2msg(fDepth)
                msgDepth.header.stamp.secs = now.secs
                msgDepth.header.stamp.nsecs = now.nsecs
                msgDepth.header.frame_id = 'camera_link'
                pubDepth.publish(msgDepth)

                msgColor = CV2msg(fBGR)
                msgColor.header.stamp.secs = now.secs
                msgColor.header.stamp.nsecs = now.nsecs
                msgDepth.header.frame_id = 'camera_link'
                pubColor.publish(msgColor)
                
            # exit while loop when press q
            ch = cv2.waitKey(g_wait_ms)
            if ch& 0xFF == 27: #ESC key
                g_IsStop = 1
                break
            elif ch& 0xFF == ord('s'):
                g_SaveDist = 1
            
            cnt=cnt+1

    else:
        print("Open camera fail!")
        
    # wait thread stop.
    t.join()
    # Release camera
    cap.release()
    # Release queue buffer
    q_rgb.put(0)
    q_d.put(0)

    # Close all OpenCV windows
    cv2.destroyAllWindows()
