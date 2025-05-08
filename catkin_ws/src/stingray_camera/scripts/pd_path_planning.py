#!/usr/bin/env python
# license removed for brevity
import rospy
# import actionlib
from enum import Enum
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
# from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from math import pow, atan, atan2, sqrt
from geometry_msgs.msg import Twist
import tf.transformations as tft

import math

# Following are the manually identified vertices of the UMass' M.
CUBICLE_X_Y_THETA = {
    "1A": (0.0, 0.0, 0.0),
    "1B": (0.0, 0.0, 0.0),
}

CUBICLE_QR_STR = {
    "1A": "1A",
    "1B": "1B",
}

ISLE_X_Y_THETA = {
    "1": (0.0, 0.0, 0.0),
}

DISPENSER_X_Y_THETA = {
    "D1": (0.0, 0.0, 0.0),
}

########################
# Robot State Constants
########################
class RobotState(Enum):
    IDLE = "idle"
    ACTIVE = "active"

ROBOT_ACTIVE_COLOR = "FFA500"
ROBOT_IDLE_COLOR = "00FF00"

class RobotController:
    
    def __init__(self):
        
        self.robot_state = RobotState.IDLE
        self.signal_states = {}
        self.pose = None

        # Publisher Declarations
        self.qr_image_pub = None
        self.led_ring_pub = None

        # Subscriber Declarations
        self.camera_image_sub = None
        self.qr_sub = None

        # Controller Node
        rospy.init_node('robot_controller', anonymous=True)
        self.initialize_signal_poses()

        # Publisher Initializations
        self.qr_image_pub = rospy.Publisher("/image", Image, queue_size=10)
        self.led_ring_pub = rospy.Publisher("/cmd_color", Int32, queue_size=10)

        # Subscriber Initializations
        self.camera_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.qr_sub = rospy.Subscriber("/barcode", String, self.qr_callback)
        
        self.rate = rospy.Rate(10) # The frequency at which the communication takes place
        
        # Path planning using PD controller

        # We control the turtle's velocity via the node's publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # The node also needs to know the current position of the turtle. 
        # We will subscribe to the pose related data
        self.pose_subscriber = rospy.Subscriber('/amcl_pose',
                                                PoseWithCovarianceStamped, 
                                                self.record_pose)
        
        # Wait until pose is initialized
        while self.pose is None:
            self.rate.sleep()

        # PD controller constants
        self.linear_vel_p_constant = .3
        self.linear_vel_d_constant = 0 # This works the best. Easy to debug too.
        self.angular_vel_p_constant = .3
        self.angular_vel_d_constant = 0 # This works the best. Easy to debug too.
        
        # Errors for differential feedback
        self.previous_x_error = 0
        self.previous_y_error = 0
        self.previous_theta_error = 0
            
        # Threshold distance and the angle to stop the PD controller
        threshold_distance = .1 # meter
        threshold_angle = math.radians(7.5) # degrees
        rotate_init_threshold_angle = math.radians(75) # degrees
        
        # Get the goal to pursue in the beginning
        goal_pose = Pose()
        
        # Set X, Y, Z position       
        goal_pose.position.x = 0
        goal_pose.position.y = .5
        goal_pose.position.z = 0
        
        dispenser_angle = math.pi/2
        
        # Set the orientation
        quaternion = self.euler_to_quaternion(0, 0, dispenser_angle)
        goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w = quaternion[0],  quaternion[1], quaternion[2], quaternion[3]
        
        result = self.move2goal(goal_pose, threshold_distance, threshold_angle, rotate_init_threshold_angle)
        try:
            while not rospy.is_shutdown():
                rospy.spin()
        except KeyboardInterrupt:
            print("Closing the rospy...")
            exit()

    def stop_robot(self):
        # Determine the velocity control
        while rospy.is_shutdown():
            vel_msg = Twist()
            
            self.velocity_publisher.publish(vel_msg)
            
            self.rate.sleep()
        
    def execute_goal(self, goal_pose):
        """
        Helper function to execute actions to go to a designated goal
        """
        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose = goal_pose

        # rospy.loginfo("Sending goal pose to Action Server")
        # self.move_base_client.send_goal(goal)
        # self.move_base_client.wait_for_result(rospy.Duration.from_sec(5.0))
        # return self.move_base_client.get_result()

    def job_path_planning(self):
        """
        Path planning function for the robot to retrieve a drink.
        """
        # TODO: Replace with actual goal pose
        dispenser_pose = Pose()
        dispenser_pose.position.x = 2.0
        dispenser_pose.position.y = 0.2
        dispenser_pose.position.z = 0
        dispenser_pose.orientation.x = 0
        dispenser_pose.orientation.y = 0
        dispenser_pose.orientation.z = 0
        dispenser_pose.orientation.w = 0

        # result = self.execute_goal(dispenser_pose)
        result = self.move2goal(dispenser_pose)

        if result == GoalStatus.SUCCEEDED:
            rospy.loginfo("Drink accquired")

            # TODO: Set goal to go back to designated spot
            target_pose = Pose()
            target_pose.position.x = 1.
            target_pose.position.y = 1.
            target_pose.position.z = 0
            target_pose.orientation.x = 0
            target_pose.orientation.y = 0
            target_pose.orientation.z = 0
            target_pose.orientation.w = 0

            # result = self.execute_goal(target_pose)
            result = self.move2goal(target_pose)

            if result == GoalStatus.SUCCEEDED:
                rospy.loginfo("Job executed successfully")
            elif result == GoalStatus.ABORTED:
                rospy.loginfo("Job aborted by robot while returning to destination")
            elif result == GoalStatus.REJECTED:
                rospy.loginfo("Job has been rejected by the Action Server while returning to destination")
        elif result == GoalStatus.ABORTED:
                rospy.loginfo("Job aborted by robot while going to dispenser")
        elif result == GoalStatus.REJECTED:
                rospy.loginfo("Job has been rejected by the Action Server while going to dispenser")

    def idle_path_planning(self):
        """
        Path planning function for when the robot is in an idle state.
        """
        # TODO: Continuously loop through positions until a job is found.
        while self.robot_state == RobotState.IDLE:
            for destination_pose in self.signal_states.values():
                self.execute_goal(destination_pose)

    def initialize_signal_poses(self):
        """
        Helper function to initialize the signal states where the robot will go to pick up new jobs.
        """
        pass

    def update_led_ring_color(self, new_color):
        """
        Helper function to change the color of the LED ring.
        """
        color_value = int(new_color, 16)
        self.led_ring_pub.publish(color_value)

    def camera_callback(self, data):
        """
        Callback that will be called when we recieve an image from the camera.
        """
        self.qr_image_pub.publish(data)

    def qr_callback(self, data):
        """
        Callback that will be called when a QR code is detected.
        """
        rospy.loginfo(data)
        if self.robot_state == RobotState.IDLE: # TODO: Update statement to also check that the data passed is a valid signal state
            self.robot_state = RobotState.ACTIVE
            self.update_led_ring_color(ROBOT_ACTIVE_COLOR)
            self.job_path_planning() # TODO: Update statement to pass pose value from signal state
        elif self.robot_state == RobotState.ACTIVE: # TODO: Update statement to also check that the data passed is a valid signal state
            self.robot_state = RobotState.IDLE
            self.update_led_ring_color(ROBOT_IDLE_COLOR)
            self.idle_path_planning()

    def record_pose(self, turtle_pose):
        """Subscriber callback"""
        self.pose = turtle_pose.pose.pose

    def quaternion_to_euler(self, x, y, z, w):
        return tft.euler_from_quaternion([x, y, z, w])
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        return tft.quaternion_from_euler(roll, pitch, yaw)
        
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) +
                    pow((goal_pose.position.y - self.pose.position.y), 2))

    def closest_angle(self, theta_diff):
        if (theta_diff % (2*math.pi)) > math.pi:
            return -(2*math.pi - (theta_diff % (2*math.pi)))
        else:
            return theta_diff % (2*math.pi)
    
    def rotate_to_match_goal(self, goal_pose, threshold_angle):
        
        # Instantiate the velocity message for controlling the bot.
        vel_msg = Twist()
    
        # 1. Rotate to face the goal
        goal_base_angle = atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)
        
        robot_pose_angle = self.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)[2]
        
        theta_diff = goal_base_angle - robot_pose_angle
        
        theta_diff = self.closest_angle(theta_diff)
        print("R1 - Theta diff", theta_diff)
        
        try:
            while abs(theta_diff) >= threshold_angle:
                
                # Determine the velocity control
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.angular.z = self.angular_vel_p_constant*theta_diff + self.angular_vel_d_constant*(theta_diff - self.previous_theta_error)
                
                self.velocity_publisher.publish(vel_msg)
                self.previous_theta_error = theta_diff
                
                self.rate.sleep()
                
                print("---")
                print("R1 - Goal Theta", goal_base_angle, "Degree:", math.degrees(goal_base_angle))
                print("R1 - Current Theta", robot_pose_angle, "Degree:", math.degrees(robot_pose_angle))
                print("R1 - Theta difference", theta_diff, "Degree:", math.degrees(theta_diff))
                print("R1 - Goal Position", goal_pose.position.x , goal_pose.position.y)
                print("R1 - Current Position", self.pose.position.x, self.pose.position.y)
                print("---")
                
                # Just in case the robot moved slightly
                goal_base_angle = atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)
                robot_pose_angle = self.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)[2]
                
                theta_diff = goal_base_angle - robot_pose_angle
                theta_diff = self.closest_angle(theta_diff)
                
        except KeyboardInterrupt:
            print("Exiting R1...")
            exit()
    
    def rotate_to_match_orientation(self, goal_pose, threshold_angle):
        vel_msg = Twist()
        
        goal_orientation = self.quaternion_to_euler(goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)[2]
        current_orientation = self.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)[2]
        theta_diff = goal_orientation - current_orientation
        theta_diff = self.closest_angle(theta_diff)
        
        self.previous_theta_error = 0
        
        try:
            while theta_diff >= threshold_angle:
                rospy.loginfo("R2 - Rotating to face the goal")
                
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.angular.z = self.angular_vel_p_constant*theta_diff + self.angular_vel_d_constant*(theta_diff - self.previous_theta_error)
                
                self.velocity_publisher.publish(vel_msg)
                self.previous_theta_error = theta_diff
                
                self.rate.sleep()
                
                goal_orientation = self.quaternion_to_euler(goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)[2]
                current_orientation = self.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)[2]
                theta_diff = goal_orientation - current_orientation
                theta_diff = self.closest_angle(theta_diff)
                
                # rospy.loginfo("R2 - Theta difference", str(theta_diff))
                print("R2 - Goal Theta", goal_orientation)
                print("R2 - Current Theta", current_orientation)
                print("R2 - Theta difference", theta_diff)
            
        except KeyboardInterrupt:
            print("Exiting R2...")
            exit()
            
    
    def move2goal(self, goal_pose, threshold_distance, threshold_angle, rotate_init_threshold_angle):
        """The function that makes turtle reach the goal position using closed loop feedback."""
        
        # Instantiate the velocity message for controlling the bot.
        vel_msg = Twist()

        # (RT)*R - (Rotate-Translate)*-Rotate
        # The bot needs to rotate to face the goal, then move towards the goal, and finally rotate to align with the goal.
        
        # 1. Rotate to face the goal
        self.rotate_to_match_goal(goal_pose, threshold_angle)
        
        # 2. Move towards the goal
        distance_diff = self.euclidean_distance(goal_pose)
        try:
            while distance_diff >= threshold_distance:
                
                # Modify the linear velocity along the plane of motion.
                vel_msg.linear.x = self.linear_vel_p_constant*distance_diff + self.linear_vel_d_constant*(distance_diff - self.previous_x_error)
                vel_msg.linear.y = 0
                vel_msg.angular.z = 0

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
                
                self.previous_x_error = distance_diff
                
                # Publish at the desired rate.
                self.rate.sleep()

                rospy.loginfo("T - Heading towards the goal")
                print("T - Goal xy", goal_pose.position.x, goal_pose.position.y)
                print("T - Current xy", self.pose.position.x, self.pose.position.y)
                print("T - Euclidean distance", distance_diff)
                
                distance_diff = self.euclidean_distance(goal_pose)
                
                # Keep track of the angle if it is increasing beyond a threshold
                goal_base_angle = atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)
                robot_pose_angle = self.quaternion_to_euler(self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)[2]
                
                theta_diff = goal_base_angle - robot_pose_angle
                theta_diff = self.closest_angle(theta_diff)
                
                if abs(theta_diff) > rotate_init_threshold_angle:
                    print("Adjusting the angle before moving ahead")
                    self.rotate_to_match_goal(goal_pose, threshold_angle)
        except KeyboardInterrupt:
            print("Exiting T...")
            exit()
        # # 3. Rotate to align with the goal
        
        self.rotate_to_match_orientation(goal_pose, threshold_angle)
        
        print("Done reaching the goal!")
           
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        
        return GoalStatus.SUCCEEDED


if __name__ == '__main__':
    
    robot_controller = RobotController()
    