#!/usr/bin/env python

###############################################################################
# Copyright (C) 2009, 2010 Soonhac Hong(sh2723@columbia.edu)
# Brief : This package publishes sensor_msgs/image from wireless camera which provides a MJPEG stream file.
# Usage : $roscore
#         $make
#         $rosrun wireless_camera wireless_camera.py
#         $rosrun image_view image_view image:=/wireless_camera/image
###############################################################################

import roslib; roslib.load_manifest('wireless_camera')
import rospy
import urllib
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class wireless_camera:
    
    def __init__(self):
        #IMPORTANT : Change self.CAMERA_IP to your camera IP
        self.CAMERA_IP = "http://128.59.19.226:1024/img/video.mjpeg"   #"http://192.168.1.2/img/video.mjpeg"
        self.image_pub = rospy.Publisher("wireless_camera/image",Image)
        self.bridge = CvBridge()
               
    def open_file(self):
        f = urllib.urlopen(self.CAMERA_IP)
    
        #read data as a string
        buffer_size = 65536;        #2^16 = 65536 : Need to optimize
        read_data = f.read(buffer_size)
        EOI = [0xff,0xD9]   #EOI[0] = 255, EOI[1] = 217
        SOI = [0xff,0xD8]   #SOI[0] = 255, SOI[1] = 216 
        
        #Find EOI and SOI
        #Convert the character of read_data to the acii code by ord(). Additionally, chr(97) will show 'a'.
        SOI_pos = 0
        SOI_pos_prev = 0
        for i in range(1,len(read_data)-1):
            if SOI[0]==ord(read_data[i:i+1]) and SOI[1]==ord(read_data[i+1:i+2]):
                if SOI_pos == 0:
                    SOI_pos=i
                else:
                    SOI_pos_prev = SOI_pos
                    SOI_pos = i
                    break       
            if EOI[0]==ord(read_data[i:i+1]) and EOI[1]==ord(read_data[i+1:i+2]):
                EOI_Pos=i
                    
        #Write the data into a temporary file.
        f=open('temp.jpg','w')
        f.write(read_data[SOI_pos_prev:SOI_pos])    # data from first SOI to the next SOI are saved to generated a JPEG file.      
    
    def publish_image(self):
        #Load temporary JPEG image
        cv_image=cv.LoadImage("temp.jpg")
        
        #Convert openCV image into ROS messages
        image_message = self.bridge.cv_to_imgmsg(cv_image, "bgr8")
        
        #Publish image message using OpenCV
        self.image_pub.publish(image_message)
    

if __name__ == '__main__':
    rospy.loginfo("Start to publish image from a wireless camera")
    wc = wireless_camera()
    rospy.init_node('wireless_camera', anonymous=True)
    while not rospy.is_shutdown():
        wc.open_file()
        wc.publish_image()