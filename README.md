
```sh
# installation
$ sudo apt-get install android-tools-adb

# cmd for using ffplay to check the UVC stream
sudo ffplay -f v4l2 -framerate 20 -video_size 640x360 -input_format yuyv422 -i /dev/video0

# basic cmd
scrcpy
adb devices
ls /dev/video*
bash 2_openapp.sh

# file detail
1_runuvctool.sh => 2_openapp.sh + pyCVTest5p.py + 3_CloseApp.sh
```

## ros utils
### topics
```bash
/Altek/depth/image_rect_raw

/Altek/color/image_raw
/AltekC/color/image_raw/compressed
```

## basic operation
```python
ret, frame_cv = cap.read()
if ret == 0 :
    print("No video frame")
    ch = cv2.waitKey(g_wait_1s)
    continue
```

## FAQ
### 1. adb devices shows “no_permissions”
(ref: https://stackoverflow.com/questions/53887322/adb-devices-no-permissions-user-in-plugdev-group-are-your-udev-rules-wrong)

```bash
# Check vid/pid

$ lsusb

# create and edit rules file
$ sudo vi /etc/udev/rules.d/51-android.rules
# add following string to “51-android.rules”
SUBSYSTEM=="usb", ATTR{idVendor}=="05c6", ATTR{idProduct}=="90cb", MODE="0666", GROUP="plugdev"

# reload rules file
$ sudo udevadm control --reload-rules
```
重新plug-in/out usb
再次adb devices 查看結果.


## Hardware setting
![](https://github.com/Leohsieh57/altek_ros/blob/master/image/setting.jpg)
