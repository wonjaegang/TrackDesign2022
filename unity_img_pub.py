#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import cv2

from sensor_msgs.msg import CompressedImage

cap = cv2.VideoCapture(0)

while True:
    # Get image frame
    success, img = cap.read()        
    
    rospy.init_node('hijaewan', anonymous=True)
    image_raw = rospy.Publisher('image_raw/compressed',CompressedImage, queue_size=1)
    imgdata = CompressedImage()

    imgdata.data = img
    imgdata.format = 'jpeg'
    image_raw.publish(imgdata)
    
    # Display
    cv2.imshow("Image", img)
    cv2.waitKey(1)
cap.release()
cv2.destroyAllWindows()