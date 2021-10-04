#!/usr/bin/env python
import rospy
import struct
import os
from rtabmap_ros.msg import UserData

def loop():
    rospy.init_node('wifi_signal_pub', anonymous=True)
    pub = rospy.Publisher('wifi_signal', UserData, queue_size=10)
    rate = rospy.Rate(0.5) # 0.5hz
    while not rospy.is_shutdown():
    
        myCmd = os.popen('nmcli dev wifi | grep "^*"').read()
        cmdList = myCmd.split()
        
        if len(cmdList) > 6:
            quality = float(cmdList[6])
            msg = UserData()
            
            # To make it compatible with c++ sub example, use dBm
            dBm = quality/2-100
            rospy.loginfo("Network \"%s\": Quality=%d, %f dBm", cmdList[1], quality, dBm)
            
            # Create user data [level, stamp].
            # Any format is accepted.
            # However, if CV_8UC1 format is used, make sure rows > 1 as 
            # rtabmap will think it is already compressed.
            msg.rows = 1
            msg.cols = 2
            msg.type = 6 # Use OpenCV type (here 6=CV_64FC1): http://ninghang.blogspot.com/2012/11/list-of-mat-type-in-opencv.html
            
            # We should set stamp in data to be able to
            # retrieve it from the saved user data as we need 
            # to get precise position in the graph afterward.
            msg.data = struct.pack(b'dd', dBm, rospy.get_time())

            pub.publish(msg)
        else:
            rospy.logerr("Cannot get info from wireless!")
        rate.sleep()

if __name__ == '__main__':
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
