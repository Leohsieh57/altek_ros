#!/bin/sh

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
dev = 202 #202: default, 1: two more camera and manual set to 1.
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
    rf = open('./lut/depth_lut.txt', 'r')
    for line in rf.readlines():
        DepthLUT_table[i] = line
        i = i+1
    rf.close()

def LoadColorLUT_Table():
    i=0
    rf = open('./lut/color_lut.txt', 'r')
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

def CheckCamera(cap, dev, cnt):
    # Check camera open status
    while not cap.isOpened():
        cv2.waitKey(g_wait_1s)
        cap = cv2.VideoCapture(dev, cv2.CAP_ANY)
        print("Wait for opening camera...")
        if cnt <= 0:
            break
    return cnt


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
    
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("th w=%d, h=%d, g_mode=%d" % (w, h, g_mode))
    
    if g_mode == 1:
        while(g_IsStop == 0):
            #tic1=time.time()
            #print("cycle-t: ",tic1)
            #tic2=tic1
        
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
            q.put(fBGR)
            
            if(g_SaveDist == 1):
                fcv = open('./Image/frame_dist.raw','wb')
                fcv.write(frame_cv)
                fcv.close()
                print("Write frame_cv.raw")
                g_SaveDist = 0
        
    else:
        while(g_IsStop == 0):
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
            fBGR = np.concatenate((fBGRnv21, fBGRdt), axis=1)
            q.put(fBGR)
            
            if(g_SaveDist == 1):
                fcv = open('./Image/frame_nv21.raw','wb')
                fcv.write(ftmp)
                fcv.close()
                fcvd = open('./Image/frame_dist.raw','wb')
                fcvd.write(fdist)
                fcvd.close()
                print("Write frame_cv.raw")
                g_SaveDist = 0


# show versions
ShowVersions()

# select camera, 0, or 1, or ...
cap = cv2.VideoCapture(dev, cv2.CAP_ANY)
ret = CheckCamera(cap, dev, cnt)

q = Queue()

if cap.isOpened():
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
    
    while(True):
        fBGR = q.get()
        cv2.namedWindow('Distance', 1)
        cv2.imshow('Distance', fBGR)
        
        # exit while loop when press q
        ch = cv2.waitKey(g_wait_ms)
        if ch& 0xFF == 27: #ESC key
            g_IsStop = 1
            break
        elif ch& 0xFF == ord('s'):
            g_SaveDist = 1;
        
        cnt=cnt+1

else:
    print("Open camera fail!")
    
# wait thread stop.
t.join()
# Release camera
cap.release()
# Release queue buffer
q.put(0)

# Close all OpenCV windows
cv2.destroyAllWindows()
