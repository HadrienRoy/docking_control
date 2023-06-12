#!/usr/bin/env python3

# detect_tag_pupil.py
#
# Author: Hadrien Roy
#
# Inputs: apriltag library from Pupil Labs
# Outputs:
#
# Purpose: To detect AprilTag

from re import M
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import cv2
import numpy as np
import pupil_apriltags

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from docking_interfaces.srv import StartAprilTagDetection, Docking


from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class DetectTagPupilNode(Node):
    def __init__(self):
        super().__init__("detect_tag_pupil")

        ### PARAMETERS ###
        self.declare_parameter('robot_id', 'tb3_1')
        self.robot_id = self.get_parameter('robot_id').value
        self.add_on_set_parameters_callback(self.set_parameters_callback)
        self.get_logger().info("New value set: %s" % self.robot_id)


        ### SUBSCRIBERS ###
        # /uncompressed not publishing in sim
        # self.image_subscriber = self.create_subscription(
        #     Image, "camera/image_raw/uncompressed", self.callback_image, qos_profile_sensor_data)
        self.image_subscriber = self.create_subscription(
            Image, "camera/image_raw", self.callback_image, qos_profile_sensor_data)


        ### PUBLISHERS and SERVICES ###
        self.detections_publisher = self.create_publisher(
            Pose, "detections", 10)
        # self.detections_publisher = self.create_publisher(
        #     AprilTagDetection, "detections", 10)

        self.start_tag_detection_service = self.create_service(
            StartAprilTagDetection, 'detect_tag_pupil/start_apriltag_detection', self.start_apriltag_detection_server)


        ### VARIABLES ###

        self.counter = 0

        # Initialize the transform broadcaster
        # self.br = TransformBroadcaster(self)
        self.br = StaticTransformBroadcaster(self)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        self.start_tag_detection = False

        # Camera parameters, {fx, fy}: focal length (px), {cx, cy}: focal center (px)
        self.fx = 495.5130372726824
        self.fy = 495.2808626986745
        self.cx = 309.64896771170527
        self.cy = 235.54576124624984
        self.tag_size = 0.24545
        self.camera_params = [self.fx, self.fy, self.cx, self.cy]

        # Define AprilTag detector
        self.detector = pupil_apriltags.Detector(families='tag36h11',
                                                 nthreads=1,
                                                 quad_decimate=1.0,
                                                 quad_sigma=0.0,
                                                 refine_edges=1,
                                                 decode_sharpening=0.25,
                                                 debug=0)

        self.get_logger().info("AprilTag Detection Node has been started.")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'robot_id' and param.type == Parameter.Type.STRING:
                self.robot_id = param.value
        return SetParametersResult


    
    def start_apriltag_detection_server(self, request, response):
        if request.service == 'start':
            self.start_tag_detection = True
            self.get_logger().info("Start AprilTag Detecttion.")

        response.success = True

        return response
    
    
    def callback_image(self, msg):
        if (self.start_tag_detection):
            self.counter += 1
            if self.counter % 10 == 0:
                self.counter = 0

                try:
                    image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                except Exception as e:
                    self.get_logger().error(e)

                # Load input image and convert to grayscale
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                detections = self.detector.detect(
                    gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

                # if detections not empty
                if len(detections) >= 1:
                    self.publish_detections_data(detections)
                

    def publish_detections_data(self, detections):
        
        # for testing
        self.start_tag_detection = True 
        # self.get_logger().info("AprilTag publish detection.")
        ####

        if (self.start_tag_detection):

            R = np.zeros((3,3))
            R[0][0] = detections[0].pose_R[0, 0]
            R[0][1] = detections[0].pose_R[0, 1]
            R[0][2] = detections[0].pose_R[0, 2]
            R[1][0] = detections[0].pose_R[1, 0]
            R[1][1] = detections[0].pose_R[1, 1]
            R[1][2] = detections[0].pose_R[1, 2]
            R[2][0] = detections[0].pose_R[2, 0]
            R[2][1] = detections[0].pose_R[2, 1]
            R[2][2] = detections[0].pose_R[2, 2]
            
            tr = np.matrix.trace(R)
            q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

            if(tr > 0):
                tr = np.sqrt(tr + 1)
                q[0] = 0.5 * tr
                tr = 0.5/tr
                q[1] = (R[2,1] - R[1,2]) * tr
                q[2] = (R[0,2] - R[2,0]) * tr
                q[3] = (R[1,0] - R[0,1]) * tr

            else:
                i = 0
                if (R[1,1] > R[0,0]):
                    i = 1
                if (R[2,2] > R[i,i]):
                    i = 2
                j = (i+1)%3
                k = (j+1)%3

                tr = np.sqrt(R[i,i] - R[j,j] - R[k,k] + 1)
                q[i] = 0.5 * tr
                t = 0.5 / tr
                q[0] = (R[k,j] - R[j,k]) * tr
                q[j] = (R[j,i] + R[i,j]) * tr
                q[k] = (R[k,i] + R[i,k]) * tr


            ### for tesing
            detections_msg = Pose()
            # q0=qw, q1=qx, q2=qy, q3=qz
            detections_msg.orientation.w = q[0]
            detections_msg.orientation.x = q[1]
            detections_msg.orientation.y = q[2]
            detections_msg.orientation.z = q[3]
            detections_msg.position.x = detections[0].pose_t[0, 0]
            detections_msg.position.y = detections[0].pose_t[1, 0]
            detections_msg.position.z = detections[0].pose_t[2, 0]
            # Send the pose of the detected tag
            self.detections_publisher.publish(detections_msg)
            # self.get_logger().info(str(detections[0].tag_id))
            ### end testing

            t = TransformStamped()

            # corresponding tf variables
            t.header.stamp = self.get_clock().now().to_msg()
            # t.header.frame_id = self.robot_id+'/camera_rgb_optical_frame'
            # t.child_frame_id = self.robot_id+"/tag_36h11_00408"
            t.header.frame_id = 'camera_rgb_optical_frame'
            t.child_frame_id = "tag_36h11_00408"

            t.transform.translation.x = detections[0].pose_t[0, 0]
            t.transform.translation.y = detections[0].pose_t[1, 0]
            t.transform.translation.z = detections[0].pose_t[2, 0]

            # q0=qw, q1=qx, q2=qy, q3=qz
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]

            # Send the transformation
            self.br.sendTransform(t)



def main(args=None):
    rclpy.init(args=args)
    node = DetectTagPupilNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
