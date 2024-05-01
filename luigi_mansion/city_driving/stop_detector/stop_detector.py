import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()
        self.publisher = self.create_publisher(Array,"/stop_sign/bounding_box",10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Stop Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        detector = StopSignDetector()
        prediction = detector.predict(image)[0]
        bounding_box = detector.predict(image)[1]
        if (prediction):
            self.get_logger().info("Stop Sign Detected")
            x_min  = bounding_box[0]
            y_min  = bounding_box[1]
            x_max  = bounding_box[2]
            y_min  = bounding_box[3]





        #TODO: 

def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()