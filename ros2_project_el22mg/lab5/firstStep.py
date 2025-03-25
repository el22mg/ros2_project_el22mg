# Exercise 1 - Display an image of the camera feed to the screen

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.receiveImageCallback, 10)

        self.sensitivity = 10
        self.bridge = CvBridge()

        self.subscription  # prevent unused variable warning
        
    def receiveImageCallback(self, data):
                
        try:
            cv2Image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError:
            pass
            
        # Convert image from BGR colourspace to HSV
        hsvColourSpaceImage = cv2.cvtColor(cv2Image, cv2.COLOR_BGR2HSV)
        
        # Create mask showing only green pixels within certain bandwidth of hue
        hsvGreenLower = np.array([60 - self.sensitivity, 100, 100])
        hsvGreenUpper = np.array([60 + self.sensitivity, 255, 255])

        greenMask = cv2.inRange(hsvColourSpaceImage, hsvGreenLower, hsvGreenUpper)
        
        # Filter image
        filteredImage = cv2.bitwise_and(hsvColourSpaceImage, hsvColourSpaceImage, mask=greenMask)
        
        # Display filtered image
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 
        cv2.imshow('camera_Feed', filteredImage)
        cv2.resizeWindow('camera_Feed', 320, 240) 
        cv2.waitKey(3)
        
        return
        

def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()

    rclpy.init(args=None)
    cI = colourIdentifier()


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()
    

# Check if the node is executing in the main path
if __name__ == '__main__':
    main()
