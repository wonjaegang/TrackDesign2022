#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage

cap = cv2.VideoCapture(2, cv2.CAP_V4L)
# cap.set(cv2.CAP_PROP_FPS, 60)
#노드 선언
rospy.init_node('img_pub', anonymous=True)
image_raw = rospy.Publisher('image_raw/compressed',CompressedImage, queue_size=1)
imgdata = CompressedImage()

rospy.sleep(1)

while True:
    #이미지 가져오기
    success, img1 = cap.read()   
    img_1 = cv2.resize(img1, dsize=(1920,1080))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps))
    img = cv2.resize(img1, dsize=(640,360), fx=1.0, fy=0.7)
    encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),60]

    #CompressedImage 메시지 작성
    imgdata.header.stamp = rospy.Time.now()
    imgdata.format = "jpg"
    imgdata.data = np.array(cv2.imencode('.jpg', img, encode_param)[1]).tostring()

    image_raw.publish(imgdata)
    
    #Publishing
    np_arr = np.fromstring(imgdata.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    #Display
    #cv2.imshow("Image", img1)
    cv2.imshow("User_View", img_1)
    
    if cv2.waitKey(1) > 0:
        # break
        pass
    #rospy.sleep(0.1)
    
cv2.destroyAllWindows()