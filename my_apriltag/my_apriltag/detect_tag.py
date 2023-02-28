#!/usr/bin/env python3

# detect_tag.py
#
# Author: Hadrien Roy
#
# Inputs: apriltag library from AprilRobotics
# Outputs:
#
# Purpose: To detect AprilTag

import rclpy
from rclpy.node import Node

from apriltag import apriltag
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import qos_profile_sensor_data

class DetectTagNode(Node):
    def __init__(self):
        super().__init__("detect_tag")

        self.counter = 0

        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.callback_image, qos_profile_sensor_data
        )

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.detector = apriltag("tag36h11")

        # Construct argument parser and pase the arguments
        # self.ap = argparse.ArgumentParser()
        # self.ap.add_argument("-i", "--image", required=True,
        #                 help="path to input image")
        # args = vars(self.ap.parse_args())
        

    def callback_image(self, msg):
        self.counter += 1

        if self.counter % 25 == 0:
            self.counter = 0

            try:
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # show_img(cv_image)
  
            except CvBridgeError as e:
                self.get_logger().error(e)
            
            
            self.get_logger().info("Loading Image...")

            # Load input image and convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Define AprilTag detector
            # self.get_logger().info("Detecting AprilTags...")
          
            detections = self.detector.detect(gray)
         
            #self.get_logger().info("total AprilTags detected" + str(detections))
            
            #err = apriltag.estimate_tag_pose()

            self.get_logger().info("Tag # " + str(detections[0]["id"]))
            


def main(args=None):
    rclpy.init(args=args)
    node = DetectTagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
