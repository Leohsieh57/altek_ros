
import cv2
import numpy as np
import time

gtestmethod = 2
gDistInMin = 200
gDistInMax = 5000
dev = 203 #202
cnt = 10
g_SaveDist = 0
g_wait_ms = 50
g_wait_1s = 1000
DepthLUT_table = [0]*1024
ColorLUT_table = [0]*1024*3
g_rgb=1

def ShowVersions():
    print('cv2 ver=%s' % cv2.__version__)
    #print('PIL ver=%s' % Image.__version__)
    print('numpy ver=%s' % np.__version__)

def opencv_cfg():
    global g_rgb
    cap.set(cv2.CAP_PROP_FORMAT, -1.0)
    #cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
    cap.set(cv2.CAP_PROP_MODE, 0.0)
    if g_rgb == 1:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 630.0)
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360.0)

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
    
    
# show versions
ShowVersions()

# select camera, 0, or 1, or ...
cap = cv2.VideoCapture(dev, cv2.CAP_ANY)

# Check camera open status
while not cap.isOpened():
    cv2.waitKey(g_wait_1s)
    cap = cv2.VideoCapture(dev, cv2.CAP_ANY)
    print("Wait for opening camera...")
    if cnt <= 0:
        break

if cap.isOpened():
    cnt = 0
    opencv_cfg()
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("1 w=%d, h=%d" % (w, h))

    #LoadDepthLUT_Table()
    LoadColorLUT_Table()
    get_opencv_cfg()
    
    table_size = gDistInMax - gDistInMin + 1
    mappinglist = [0]*65535 #table_size
    for i in range(0, table_size):
        mappinglist[i] = round(i * 1023 / (gDistInMax - gDistInMin))
    # converting list to array
    mapping = np.array(mappinglist)
    ColorLUT_table_np = np.array(ColorLUT_table)
    #tic1 = tic2 = 0
    
    while(True):
        #tic=time.time()
        # get 1 image from camera
        ret, frame_cv = cap.read()
        if ret == 0 :
            print("No video frame")
            ch = cv2.waitKey(g_wait_1s)
            continue
        #if gtestmethod == 4 :
        if g_rgb == 1:
            imgSize = (h*2,w,1)
            frame_np = np.frombuffer(frame_cv, dtype=np.uint8).reshape(imgSize)
            h_rgb=540
            ftmp = frame_np[:h_rgb,:w]
            fBGR = cv2.cvtColor(ftmp,cv2.COLOR_YUV2BGR_NV21)
            cv2.namedWindow('RGB', 1)
            cv2.imshow('RGB', fBGR)
        else:
            imgSize = (w,h)
            frame_np = np.frombuffer(frame_cv, dtype=np.uint16).reshape(imgSize)
            fBGR = np.zeros((h,w,3), np.uint8)
            PixelCvt_Dist2RGBnp(frame_np, w, h, fBGR, gDistInMin, gDistInMax, mapping)
            cv2.namedWindow('Distance', 1)
            cv2.imshow('Distance', fBGR)

        # exit while loop when press q
        ch = cv2.waitKey(g_wait_ms)
        #if ch& 0xFF == ord('q'):
        if ch& 0xFF == 27: #ESC key
            break
        elif ch& 0xFF == ord('s'):
            g_SaveDist = 1;

        if(g_SaveDist == 1):
            if g_rgb == 1:
                fcv = open('./Image/frame_rgb.raw','wb')
                fcv.write(ftmp)
            else:
                fcv = open('./Image/frame_cv.raw','wb')
                fcv.write(frame_cv)
            fcv.close()
            print("Write frame_cv.raw")
            g_SaveDist = 0
        cnt=cnt+1

else:
    print("Open camera fail!")

# Release camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
