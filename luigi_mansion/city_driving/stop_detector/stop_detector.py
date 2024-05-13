import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from .detector import StopSignDetector
from std_msgs.msg import Header
from ackermann_msgs.msg import AckermannDrive,AckermannDriveStamped
import time

class SignDetector(Node):
    def __init__(self):
        super().__init__("stop_detector")
        self.detector = StopSignDetector()
        self.publisher = self.create_publisher(Point,"/stop_sign/center_pixel",10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 2)
        self.drive_pub=self.create_publisher(AckermannDriveStamped,"/vesc/low_level/input/safety",1)
        self.bridge = CvBridge()

        self.last_stopped=None

        self.get_logger().info("Stop Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        # if self.last_stopped!=None:
            
        #     if time.time()-self.last_stopped<=1:
        #         header=Header()
        #         header.stamp=self.get_clock().now().to_msg()
        #         header.frame_id="base_link"
        #         drive=AckermannDrive()
        #         drive.steering_angle=0.0
        #         drive.steering_angle_velocity=0.0
        #         drive.speed=0.0
        #         drive.acceleration=0.0
        #         drive.jerk=0.0
        #         stamped_msg=AckermannDriveStamped()
        #         stamped_msg.header=header
        #         stamped_msg.drive=drive
        #         self.drive_pub.publish(stamped_msg)

        #     if time.time()-self.last_stopped<=5:
        #         return

        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        detector = StopSignDetector()
        prediction,bounding_box = detector.predict(image)
        if prediction:
            self.get_logger().info("Stop Sign Detected")
            # self.last_stopped=time.time()
            # header=Header()
            # header.stamp=self.get_clock().now().to_msg()
            # header.frame_id="base_link"
            # drive=AckermannDrive()
            # drive.steering_angle=0.0
            # drive.steering_angle_velocity=0.0
            # drive.speed=0.0
            # drive.acceleration=0.0
            # drive.jerk=0.0
            # stamped_msg=AckermannDriveStamped()
            # stamped_msg.header=header
            # stamped_msg.drive=drive
            # self.drive_pub.publish(stamped_msg)
            x_min  = bounding_box[0]
            y_min  = bounding_box[1]
            x_max  = bounding_box[2]
            y_max  = bounding_box[3]
            x_center = int((x_min+x_max)/2)
            y_center = int((y_min+y_max)/2)
            msg = Point()
            msg.x = x_center
            msg.y = y_center
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    detector = SignDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()