#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class YellowCubeDetector(Node):
    def __init__(self):
        super().__init__('yellow_cube_detector')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/yellow_cube_detection/image',
            10)

        self.marker_pub = self.create_publisher(
            Marker,
            '/yellow_cube_marker',
            10)

        # Publisher for the pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/yellow_cube_pose',
            10)

        # Variables to store the latest messages
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_info = None

    def depth_callback(self, depth_msg):
        self.latest_depth_image = depth_msg

    def camera_info_callback(self, info_msg):
        self.camera_info = info_msg

    def image_callback(self, img_msg):
        if self.latest_depth_image is None:
            self.get_logger().warn('No depth image received yet.')
            return

        if self.camera_info is None:
            self.get_logger().warn('No camera info received yet.')
            return

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Convert depth image to OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding='passthrough')

        # Check depth image encoding and convert to float32 if necessary
        if depth_image.dtype != np.float32:
            if depth_image.dtype == np.uint16:
                # Assuming depth image is in millimeters, convert to meters
                depth_image = depth_image.astype(np.float32) / 1000.0
            else:
                depth_image = depth_image.astype(np.float32)

        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create a mask for yellow color
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the centroid of the largest contour
            M = cv2.moments(largest_contour)
            if M['m00'] == 0:
                self.get_logger().warn('Zero division error in moments calculation.')
                return

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Draw the contour and centroid on the image
            cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

            # Get depth value at the centroid
            depth_value = depth_image[cy, cx]

            # Check for valid depth value
            if np.isfinite(depth_value) and depth_value > 0:
                # Get camera intrinsic parameters
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx_i = self.camera_info.k[2]
                cy_i = self.camera_info.k[5]

                # Compute the 3D coordinates
                x = (cx - cx_i) * depth_value / fx
                y = (cy - cy_i) * depth_value / fy
                z = float(depth_value)

                self.get_logger().info(f'3D Coordinates of the yellow cube: x={x:.3f}, y={y:.3f}, z={z:.3f}')

                # Create a marker message
                marker = Marker()
                marker.header.frame_id = self.camera_info.header.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'yellow_cube'
                marker.id = 0
                marker.type = Marker.CUBE  # Represent the cube as a cube marker
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.05  # Adjust the scale to match the cube size
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0  # Fully opaque
                marker.color.r = 1.0  # Yellow color
                marker.color.g = 1.0
                marker.color.b = 0.0

                # Publish the marker
                self.marker_pub.publish(marker)

                # Create and publish the PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = self.camera_info.header.frame_id
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0

                # Publish the pose
                self.pose_pub.publish(pose_msg)

            else:
                self.get_logger().warn('Invalid depth value at the centroid.')
        else:
            self.get_logger().info('No yellow cube detected.')

        # Publish the output image
        out_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(out_img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YellowCubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
