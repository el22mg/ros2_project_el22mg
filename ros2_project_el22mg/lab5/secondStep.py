# Exercise 2 - detecting two colours, and filtering out the third colour and background.



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

        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        # We covered which topic to subscribe to should you wish to receive image data
        self.sensitivity = 10
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning
        

    def callback(self, data):

        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError:
            return
        
        cv2.namedWindow('basic_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('basic_camera_Feed', image)
        cv2.resizeWindow('basic_camera_Feed',320,240)
        cv2.waitKey(3)
        
        
        # Set the upper and lower bounds for the colours you wish to identify
        hsv_red_lower1 = np.array([0, 100, 100]) # 0~sensitivity range
        hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
        hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100]) #(180-sensitivity)~180 range
        hsv_red_upper2 = np.array([180, 100, 100])
                
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        redMask1 = cv2.inRange(Hsv_image, hsv_red_lower1, hsv_red_upper1)
        redMask2 = cv2.inRange(Hsv_image, hsv_red_lower2, hsv_red_upper2)
        greenMask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        blueMask = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)

        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two images at once, so multiple calls are necessary for more than two colours
        redMask = cv2.bitwise_or(redMask1, redMask2)
        rgMask = cv2.bitwise_or(redMask, greenMask)
        rgbMask = cv2.bitwise_or(rgMask, blueMask)

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to...
        # bitwise and an image with itself and pass the mask to the mask parameter (rgb_image,rgb_image, mask=mask)
        # As opposed to performing a bitwise_and on the mask and the image.
        rgbMaskedImage = cv2.bitwise_and(Hsv_image, Hsv_image, mask=rgbMask)
        rMaskedImage = cv2.bitwise_and(Hsv_image, Hsv_image, mask=redMask)
        gMaskedImage = cv2.bitwise_and(Hsv_image, Hsv_image, mask=greenMask)
        bMaskedImage = cv2.bitwise_and(Hsv_image, Hsv_image, mask=blueMask)
        rgMaskedImage = cv2.bitwise_and(Hsv_image, Hsv_image, mask=rgMask) # 2-colour combined mask for Jnana's sake
        
        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        
        # RGB
        cv2.namedWindow('rgb_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('rgb_camera_Feed', rgbMaskedImage)
        cv2.resizeWindow('rgb_camera_Feed',320,240)
        cv2.waitKey(3)
        
        # R
        cv2.namedWindow('r_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('r_camera_Feed', rMaskedImage)
        cv2.resizeWindow('r_camera_Feed',320,240)
        cv2.waitKey(3)
        
        # G
        cv2.namedWindow('g_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('g_camera_Feed', gMaskedImage)
        cv2.resizeWindow('g_camera_Feed',320,240)
        cv2.waitKey(3)
        
        # B
        cv2.namedWindow('b_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('b_camera_Feed', bMaskedImage)
        cv2.resizeWindow('b_camera_Feed',320,240)
        cv2.waitKey(3)
        
        # RG
        cv2.namedWindow('rg_camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('rg_camera_Feed', rgMaskedImage)
        cv2.resizeWindow('rg_camera_Feed',320,240)
        cv2.waitKey(3)
              
   


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()

    
    # Instantiate your class
    # And rclpy.init the entire node
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
