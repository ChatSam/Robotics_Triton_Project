#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
from enum import Enum
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus

########################
# Robot State Constants
########################
class RobotState(Enum):
    IDLE = "idle"
    ACTIVE = "active"

ROBOT_ACTIVE_COLOR = "FFA500"
ROBOT_IDLE_COLOR = "00FF00"

class RobotController:
    robot_state = RobotState.IDLE
    signal_states = {}

    # Publisher Declarations
    qr_image_pub = None
    led_ring_pub = None

    # Subscriber Declarations
    camera_image_sub = None
    qr_sub = None

    # Path Planning Action Client
    move_base_client = None

    def execute_goal(self, goal_pose):
        """
        Helper function to execute actions to go to a designated goal
        """
        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose = goal_pose

        rospy.loginfo("Sending goal pose to Action Server")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result(rospy.Duration.from_sec(5.0))
        return self.move_base_client.get_result()

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

        result = self.execute_goal(dispenser_pose)

        if result == GoalStatus.SUCCEEDED:
            rospy.loginfo("Drink accquired")

            # TODO: Set goal to go back to designated spot
            target_pose = Pose()
            target_pose.position.x = 2.0
            target_pose.position.y = 0.2
            target_pose.position.z = 0
            target_pose.orientation.x = 0
            target_pose.orientation.y = 0
            target_pose.orientation.z = 0
            target_pose.orientation.w = 0

            result = self.execute_goal(target_pose)

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
            
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.initialize_signal_poses()

        # Publisher Initializations
        self.qr_image_pub = rospy.Publisher("/image", Image, queue_size=10)
        self.led_ring_pub = rospy.Publisher("/cmd_color", Int32, queue_size=10)

        # Subscriber Initializations
        self.camera_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.qr_sub = rospy.Subscriber("/barcode", String, self.qr_callback)

        # Client Initializations
        self.move_base_client = actionlib.SimpleActionClient('pose_base_controller', MoveBaseAction)
        wait = self.move_base_client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")

        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    robot_controller = RobotController()
