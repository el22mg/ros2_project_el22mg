import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Lab5Listener(Node):
    def __init__(self):
        super().__init__('lab5Listener')
        self.subscription = self.create_subscription(String, 'greenObjectDetected', self.listener_callback, 10)
        self.subscriptionFourthStep = self.create_subscription(String, 'fourthStepDebug', self.listenerFourthStepCallback, 10)
        
        # prevent unused variable warnings
        self.subscription  
        self.subscriptionFourthStep

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data!r}')

    def listenerFourthStepCallback(self, msg):
    	self.get_logger().info(f'FS: {msg.data!r}')


def main(args=None):
    rclpy.init(args=args)
    lab5Listener = Lab5Listener()
    rclpy.spin(lab5Listener)


if __name__ == '__main__':
    main()
