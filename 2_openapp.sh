#!/bin/sh

adb devices
adb kill-server
adb start-server
adb wait-for-device

# Check boot ready and enter main screen.
status=0
for cnt in $(seq 1 20)
do
    status=0
    rm "check.txt"
    sleep 2s
    adb shell getprop dev.bootcomplete > check.txt
    filename='check.txt'
    while IFS='' read -r line || [ -n "$line" ]; do
        #echo $cnt
        #echo $line
        if [ $line -eq '1' ]; then
            #echo "pass"
            status=1
        fi
    done < $filename
    if [ $status -eq '1' ]; then
        break;
    fi
    echo 'Wait to open camera...'
    sleep 3s
done
if [ $status -eq '0' ]; then
    echo 'Open camera failed...'
    exit 1
fi
#read -p "Press 'Enter' to continue... " var_name

# Open alcamera app.
adb root
adb wait-for-device
adb remount
adb shell setenforce 0

adb shell input keyevent 4
adb shell input keyevent 4
adb shell input keyevent 4
adb shell am start com.example.land.altek.alcamera/com.example.land.altek.alcamera.MainActivity
sleep 2
adb shell input tap 1784 758
sleep 0.5s
adb shell input tap 1402 350
sleep 0.5s
adb shell input tap 756 302
sleep 0.5s
adb shell input tap 756 712
sleep 1s
adb shell input tap 1480 202
sleep 1s
adb shell input tap 1214 892
sleep 7s

# Reset frame.en flag.
adb shell setprop persist.al.frame.en 0

# Check distance preview streaming on.
for cnt in $(seq 1 8)
do
    status=0
    #rm "camstatus.txt"
    echo 'Check distance preview on...'
    sleep 3s
    adb shell getprop persist.al.frame.en > camstatus.txt  #echo '1' > check.txt
    filename='camstatus.txt'
    while IFS='' read -r line || [ -n "$line" ]; do
        #echo $cnt
        #echo $line
        if [ $line -eq '1' ]; then
            #echo "pass"
            status=1
        fi
    done < $filename
    if [ $status -eq '1' ]; then
        break;
    fi
done
if [ $status -eq '0' ]; then
    echo 'Camera preview failed...'
    exit 1
fi

# pause (if need)
read -p "Press 'Enter' to continue... " var_name

exit 0

