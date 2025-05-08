#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return
    cv2.imwrite("green_frame.png", frame)

rospy.init_node('to_puck', anonymous=True)
r = rospy.Rate(60)

def main():
    global bridge
    bridge = CvBridge()
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    while not rospy.is_shutdown():
   	pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

