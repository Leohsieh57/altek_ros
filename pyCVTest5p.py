
import cv2
#from PIL import Image
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


def ShowVersions():
    print('cv2 ver=%s' % cv2.__version__)
    #print('PIL ver=%s' % Image.__version__)
    print('numpy ver=%s' % np.__version__)

def opencv_cfg():
    cap.set(cv2.CAP_PROP_FORMAT, -1.0)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_MODE, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

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
    depthlut = np.zeros((w,h), np.uint16)
    depthlut = 1023 - mapping[Inframe - InMin]
    ftmp=np.reshape(pRGB, (w,h,3))
    
    ftmp[:,:,0]= ColorLUT_table_np[depthlut[:,:]*3+2] # B
    ftmp[:,:,1]= ColorLUT_table_np[depthlut[:,:]*3+1] # G
    ftmp[:,:,2]= ColorLUT_table_np[depthlut[:,:]*3]   # R
    
    #ftmp[Inframe < InMin, 0] = 0 # B
    #ftmp[Inframe < InMin, 1] = 0 # G
    #ftmp[Inframe < InMin, 2] = 0 # R
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
    imgSize = (w,h)

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
        #tic2=time.time()    
        #if gtestmethod == 4 :
        frame_np = np.frombuffer(frame_cv, dtype=np.uint16).reshape(imgSize)
        #tic3=time.time()
        fBGR = np.zeros((h,w,3), np.uint8)
        #tic4=time.time()
        PixelCvt_Dist2RGBnp(frame_np, w, h, fBGR, gDistInMin, gDistInMax, mapping)
        #print(frame_np.shape, frame_np.ndim, frame_np.size)
        #print(fBGR.shape, fBGR.ndim, fBGR.size) # 691200
        #tic5=time.time()
        cv2.namedWindow('Distance', 1)
        cv2.imshow('Distance', fBGR)
        #tic6=time.time()
        # exit while loop when press q
        ch = cv2.waitKey(g_wait_ms)
        #if ch& 0xFF == ord('q'):
        if ch& 0xFF == 27: #ESC key
            break
        elif ch& 0xFF == ord('s'):
            g_SaveDist = 1;
        #tic7=time.time()
        if(g_SaveDist == 1):
            fcv = open('./Image/frame_cv.raw','wb')
            fcv.write(frame_cv)
            fcv.close()
            print("Write frame_cv.raw")
            g_SaveDist = 0
        #tic8=time.time()
        cnt=cnt+1
        #tic9=time.time()
        #print(tic1, tic2, tic3, tic4, tic5, tic6, tic7, tic8, tic9)

else:
    print("Open camera fail!")

# Release camera
cap.release()

# Close all OpenCV windows
cv2.destroyAllWindows()
