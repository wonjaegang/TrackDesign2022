#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

cap = cv2.VideoCapture(0)

#노드 선언
rospy.init_node('img_pub', anonymous=True)
image_raw = rospy.Publisher('image_raw/compressed',CompressedImage, queue_size=1)
imgdata = CompressedImage()

while True:
    #이미지 가져오기
    success, img = cap.read()   

    #CompressedImage 메시지 작성
    imgdata.header.stamp = rospy.Time.now()
    imgdata.format = "jpeg"
    imgdata.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()

    image_raw.publish(imgdata)
    
    #Publishing
    np_arr = np.fromstring(imgdata.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #Display
    cv2.imshow("Image", img)
    cv2.imshow("converted", image_np)
    cv2.waitKey(1)
cap.release()
cv2.destroyAllWindows()