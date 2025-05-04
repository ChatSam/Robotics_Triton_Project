#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

pixels = 1000
min_color = np.array([162, 160, 0])
max_color = np.array([172, 255, 255])
tol = 10
x_shift = None
x_last = None
c_shift = 200
rag = 100
angle_vel = 0.5
k_p = 0.002

def image_callback(msg):
    global x_shift
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", e)
        return
    y_shape, x_shape, _ = frame.shape
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, min_color, max_color)
    if np.sum(m != 0) < pixels:
        x_shift = None
        return
    _, x = np.where(m != 0)
    x_mean = np.mean(x, dtype=np.uint32)
    x_shift = x_shape//2 - x_mean
    x_last = x_shift


rospy.init_node('to_puck', anonymous=True)
vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
r = rospy.Rate(60)

def image_listener():
    global bridge
    bridge = CvBridge()
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    while not rospy.is_shutdown():
        ### new code here
        twist = Twist()
        if x_shift is None:
            print("Detecting")
            # If no object detected, rotate in last known direction
            if x_last is not None:
                twist.angular.z = -angle_vel if x_last < 0 else angle_vel
            else:
                twist.angular.z = angle_vel
            twist.linear.x = 0.0
        else:
            print("Adjusting")
            twist.angular.z = k_p * x_shift  # proportional angular control

            # Move forward only if relatively aligned
            if abs(x_shift) < 100:
                twist.linear.x = 0.5
            else:
                twist.linear.x = 0.0

        vel_publisher.publish(twist)
        ### new code ends here
        r.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    image_listener()
