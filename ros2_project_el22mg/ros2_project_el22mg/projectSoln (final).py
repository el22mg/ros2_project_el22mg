import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos



# CV-Relevant Libraries

import threading
import sys, time
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import signal






class ProjectSoln(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goalQueue = []
        self.navNotFinished = True
        
        
        
        # CV INIT CODE
        
        # Initialise a publisher to publish messages to the robot base
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10) # 10 Hz

        # Initialise any flags that signal a colour has been detected (default to false)
        self.blueDetected = False
        self.firstBlueDetected = False
        self.firstRedDetected = False
        self.firstGreenDetected = False

        # Initialise movement flags
        self.moveForwardsFlag = False
        self.moveBackwardsFlag = False
        self.stopFlag = False

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10

        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
        self.moveForwardsTwist = Twist()
        self.moveForwardsTwist.linear.x = 0.2

        self.stopTwist = Twist()


        # Initialise a CvBridge() and set up a subscriber to the image_raw topic
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # Publisher to talker of state feedback for debugging
        self.publisherTalker = self.create_publisher(String, 'fourthStepDebug', 10)


        
    def callback(self, data):

        # Convert the received image into an opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError:
            return

        cv2.namedWindow('basic_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('basic_camera_Feed', image)
        cv2.resizeWindow('basic_camera_Feed',320,240)
        cv2.waitKey(3)
        

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        
        hsv_red_lower1 = np.array([0, 100, 100])
        hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180-self.sensitivity, 100, 100])
        hsv_red_upper2 = np.array([180, 255, 255])
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        redMask1 = cv2.inRange(Hsv_image, hsv_red_lower1, hsv_red_upper1)
        redMask2 = cv2.inRange(Hsv_image, hsv_red_lower2, hsv_red_upper2)
        redMask = cv2.bitwise_or(redMask1, redMask2)
        
        blueMask = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        greenMask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        redMaskedImg = cv2.bitwise_and(Hsv_image, Hsv_image, mask=redMask)
        blueMaskedImg = cv2.bitwise_and(Hsv_image, Hsv_image, mask=blueMask)
        greenMaskedImg = cv2.bitwise_and(Hsv_image, Hsv_image, mask=greenMask)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        redContours, _ = cv2.findContours(redMask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        blueContours, _ = cv2.findContours(blueMask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        greenContours, _ = cv2.findContours(greenMask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)


        # Default detection of blue to false before testing
        self.blueDetected = False


        if (not self.firstRedDetected):
            # Loop over the contours
            if len(redContours)>0:

                # There are a few different methods for identifying which contour is the biggest
                # Loop through the list and keep track of which contour is biggest or
                # Use the max() method to find the largest contour
                
                c = max(redContours, key=cv2.contourArea)

                #Check if the area of the shape you want is big enough to be considered
                # If it is then change the flag for that colour to be True(1)
                if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>
                    # Alter the value of the flag
                    self.firstRedDetected = True
                    # Print indication of detection
                    self.get_logger().info(f'RED DETECTED! :0')


        # Loop over the blue contours
        if len(blueContours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(blueContours, key=cv2.contourArea)

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > 2500: #<What do you think is a suitable area?>
                # Alter the value of the flag
                self.blueDetected = True
                
                if (not self.firstBlueDetected):
                    # Alter value of 'first detection' flag
                    self.firstBlueDetected = True
                    # Print indication of detection
                    self.get_logger().info(f'BLUE DETECTED! :)')


        if (not self.firstGreenDetected):
            # Loop over the green contours
            if len(greenContours)>0:

                # There are a few different methods for identifying which contour is the biggest
                # Loop through the list and keep track of which contour is biggest or
                # Use the max() method to find the largest contour
                
                c = max(greenContours, key=cv2.contourArea)

                #Check if the area of the shape you want is big enough to be considered
                # If it is then change the flag for that colour to be True(1)
                if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>
                    # Alter the value of the flag
                    self.firstGreenDetected = True
                    # Print indication of detection
                    self.get_logger().info(f'GREEN DETECTED! :P')


            
        else: # Move towards blue object (within certain distance range)
            self.stopFlag = False
        
            if self.blueDetected:
                if cv2.contourArea(c) > 670000:
                    # Too close to object, need to move backwards
                    # Set a flag to tell the robot to move backwards when in the main loop
                    self.moveBackwardsFlag = True
                    self.moveForwardsFlag = False
                    
                elif cv2.contourArea(c) < 520000:
                    # Too far away from object, need to move forwards
                    # Set a flag to tell the robot to move forwards when in the main loop
                    self.moveForwardsFlag = True
                    self.moveBackwardsFlag = False
                    
                else:
                    self.moveBackwardsFlag = False
                    self.moveForwardsFlag = False
                    
            else:
                self.moveBackwardsFlag = False
                self.moveForwardsFlag = False
                

        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

        cv2.namedWindow('redMask_camera_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('redMask_camera_Feed', redMaskedImg)
        cv2.resizeWindow('redMask_camera_feed', 320, 240)
        cv2.waitKey(3)
        
        cv2.namedWindow('blueMask_camera_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('blueMask_camera_Feed', blueMaskedImg)
        cv2.resizeWindow('blueMask_camera_feed', 320, 240)
        cv2.waitKey(3)

        cv2.namedWindow('greenMask_camera_feed', cv2.WINDOW_NORMAL)
        cv2.imshow('greenMask_camera_Feed', greenMaskedImg)
        cv2.resizeWindow('greenMask_camera_feed', 320, 240)
        cv2.waitKey(3)

        # Publish state for debugging
        #msg = String()
        #msg.data = f'4thS Robot State: self.blueDetected = {self.blueDetected}, self.redDetected = {self.redDetected}, self.stopFlag = {self.stopFlag}, self.moveBackwardsFlag = {self.moveBackwardsFlag}, self.moveForwardsFlag = {self.moveForwardsFlag}'
        #self.publisherTalker.publish(msg)  
        
            
              
            
    def walk_forward(self):
        # Make the robot move forwards
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.03
        self.publisher.publish(desired_velocity)
        self.get_logger().info(f'Walking forwards')

    def walk_backward(self):
        # Make the robot move backwards
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.03
        self.publisher.publish(desired_velocity)
        self.get_logger().info(f'Walking backwards')

    def stop(self):
        # Make the robot stop
        desired_velocity = Twist()
        self.publisher.publish(desired_velocity)  
        self.get_logger().info(f'Stopping')      
            

        
        
        
        
        


    def queueGoals(self):
        # Add goal to queue or start immediately if idle
        self.send_goal(-5.00, 0.00, 0.00247)
        self.goalQueue.append((7.00, -7.00, 0.00247))
        self.goalQueue.append((-3.00, -11.50, 1.50))
        
        
        

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        
        # If there are more goals in the queue, send the next one
        if self.goalQueue:
            nextGoal = self.goalQueue.pop(0)
            self.send_goal(*nextGoal) # Using `*` operator unpacks tuple elements as parameters
        else:
            self.navNotFinished = False
            
            
            
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.

def main(args=None):
    rclpy.init(args=args)
    
    def signal_handler(sig, frame):
        projectSoln.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()


    projectSoln = ProjectSoln()
    
    projectSoln.queueGoals()
    
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(projectSoln,), daemon=True)
    thread.start()
    
    
    # Stall until all cube colours detected, then start moving in 1m range of blue cube
    while projectSoln.navNotFinished:
            pass

    projectSoln.get_logger().info(f'Starting to seek blue cube! Wooooooo :D')
    
    while True: # Publish moves
        
        #projectSoln.get_logger().info(f'blueDetected: {projectSoln.blueDetected}')
        #projectSoln.get_logger().info(f'redDetected: {projectSoln.redDetected}')
        #projectSoln.get_logger().info(f'stopFlag: {projectSoln.stopFlag}')
        #projectSoln.get_logger().info(f'moveForwardsFlag: {projectSoln.moveForwardsFlag}')
        #projectSoln.get_logger().info(f'moveBackwardsFlag: {projectSoln.moveBackwardsFlag}')
        #projectSoln.get_logger().info(f'moveBackwardsFlag: {projectSoln.blueContourSize}')
        
        if projectSoln.moveForwardsFlag:
            projectSoln.walk_forward()
        
        elif projectSoln.moveBackwardsFlag:
            projectSoln.walk_backward()
            
        else: # Neither move forwards or move backwards flag true
            projectSoln.stop()




if __name__ == '__main__':
    main()


# GridY: [-9, 7]
# GridX: [-13, 4]

# ros2 launch turtlebot3_gazebo turtlebot3_task_world_2025.launch.py
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_el22mg/map/map.yaml
# ros2 run ros2_project_el22mg project_soln

