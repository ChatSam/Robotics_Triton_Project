import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import time 

class Striker:
    
    def __init__(self):
        rospy.init_node('vision_dribble_omni_fsm')
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(20)

        
    
    def drive(self, x=0.0, y=0.0, z=0.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.cmd_pub.publish(twist)
        
        # for x_vel in range(int(x)+1):  # Publish multiple times
        #     twist.linear.x = float(x_vel)
        #     self.cmd_pub.publish(twist)
        #     rospy.sleep(0.1)
        
        
    def stop(self):
        self.drive(0.0, 0.0, 0.0)
        
    
    def strike(self):
        x = 3
        y = 0
        z = 0

        self.drive(x,y,z)
        rospy.sleep(1)
        self.stop()
    
    
    def run_main(self):
        n_iters = 0
        while not rospy.is_shutdown():
            rospy.loginfo(f"Iteration: {n_iters}")
            self.strike()
            n_iters += 1 
            
            if n_iters > 100:
                break
            #break
        
        

if __name__ == "__main__":
    strkr = Striker()
    time.sleep(1)
    strkr.run_main()  

