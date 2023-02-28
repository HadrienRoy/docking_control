#!/usr/bin/env python3

# detect_tag.py
#
# Author: Hadrien Roy
#
# Inputs: apriltag library from SwatBotics
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

class DetectTagSwatNode(Node):
    def __init__(self):
        super().__init__("detect_tag_swat")

        self.counter = 0

        self.image_subscriber = self.create_subscription(
            Image, "/camera/image_raw", self.callback_image, 1
        )

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Construct argument parser and pase the arguments
        # self.ap = argparse.ArgumentParser()
        # self.ap.add_argument("-i", "--image", required=True,
        #                 help="path to input image")
        # args = vars(self.ap.parse_args())
        






    def callback_image(self, msg):
        self.counter += 1
        if self.counter % 50 == 0:
            
            try:
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # show_img(cv_image)
  
            except CvBridgeError as e:
                print(e)
            
            
            self.get_logger().info("Loading Image...")

            # Load input image and convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Define AprilTag detector
            self.get_logger().info("Detecting AprilTags...")
            options = apriltag.DetectorOptions(families='tag36_11')
            detector = apriltag.Detector(options)
            results = detector.detect(gray)
            self.get_logger().info("total AprilTags detected".format(len(results)))
            
            # loop over the AprilTag detection results
            for r in results:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                cv2.line(image, ptA, ptB, (0, 255, 0), 2)
                cv2.line(image, ptB, ptC, (0, 255, 0), 2)
                cv2.line(image, ptC, ptD, (0, 255, 0), 2)
                cv2.line(image, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(r.center[0]), int(r.center[1]))
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                # draw the tag family on the image
                tagFamily = r.tag_family.decode("utf-8")
                cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print("[INFO] tag family: {}".format(tagFamily))
            # show the output image after AprilTag detection
            cv2.imshow("Image", image)
            cv2.waitKey(0)


def main(args=None):
    rclpy.init(args=args)
    node = DetectTagSwatNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
