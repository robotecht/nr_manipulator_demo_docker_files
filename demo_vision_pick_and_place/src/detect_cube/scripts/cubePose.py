#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs

class CubePoseTransformer(Node):
    def __init__(self):
        super().__init__('cube_pose_transformer')

        # Subscribe to /yellow_cube_pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/yellow_cube_pose',
            self.cube_pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for the transformed pose
        self.transformed_pose_publisher = self.create_publisher(
            PoseStamped,
            '/cube_pose_in_base_frame',
            10)

        # Publisher for the marker
        self.marker_publisher = self.create_publisher(
            Marker,
            'cube_pose_marker',
            10)

        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('CubePoseTransformer node has been started.')

    def cube_pose_callback(self, msg):
        # We have received the cube pose in camera frame
        cube_pose_in_camera_frame = msg

        self.get_logger().info('Received cube pose in camera frame:')
        self.get_logger().info(f'Position - x: {cube_pose_in_camera_frame.pose.position.x}, '
                               f'y: {cube_pose_in_camera_frame.pose.position.y}, '
                               f'z: {cube_pose_in_camera_frame.pose.position.z}')
        self.get_logger().info(f'Orientation - x: {cube_pose_in_camera_frame.pose.orientation.x}, '
                               f'y: {cube_pose_in_camera_frame.pose.orientation.y}, '
                               f'z: {cube_pose_in_camera_frame.pose.orientation.z}, '
                               f'w: {cube_pose_in_camera_frame.pose.orientation.w}')

        # Transform pose from camera frame to robot base frame
        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base',  # Target frame (robot base frame)
                cube_pose_in_camera_frame.header.frame_id,  # Source frame (camera frame)
                rclpy.time.Time())
            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(
                cube_pose_in_camera_frame.pose,
                transform)

            # Create a new PoseStamped message with the transformed pose
            cube_pose_in_base_frame = PoseStamped()
            cube_pose_in_base_frame.header.frame_id = 'link_base'  # Target frame
            cube_pose_in_base_frame.header.stamp = self.get_clock().now().to_msg()
            cube_pose_in_base_frame.pose = transformed_pose

            self.get_logger().info('Cube pose transformed to robot base frame:')
            self.get_logger().info(f'Position - x: {cube_pose_in_base_frame.pose.position.x}, '
                                   f'y: {cube_pose_in_base_frame.pose.position.y}, '
                                   f'z: {cube_pose_in_base_frame.pose.position.z}')
            self.get_logger().info(f'Orientation - x: {cube_pose_in_base_frame.pose.orientation.x}, '
                                   f'y: {cube_pose_in_base_frame.pose.orientation.y}, '
                                   f'z: {cube_pose_in_base_frame.pose.orientation.z}, '
                                   f'w: {cube_pose_in_base_frame.pose.orientation.w}')

            # Publish the transformed pose
            self.transformed_pose_publisher.publish(cube_pose_in_base_frame)

            # Create and publish the marker
            marker = Marker()
            marker.header.frame_id = 'link_base'  # Robot base frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'cube_pose'
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = cube_pose_in_base_frame.pose

            # Set the scale of the marker (cube size)
            marker.scale.x = 0.05  # 5 cm
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Set the color (red)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha (opacity)

            # Publish the marker
            self.marker_publisher.publish(marker)

            self.get_logger().info('Published cube marker in RViz.')

        except tf2_ros.LookupException as ex:
            self.get_logger().error(f'LookupException: {ex}')
        except tf2_ros.ConnectivityException as ex:
            self.get_logger().error(f'ConnectivityException: {ex}')
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().error(f'ExtrapolationException: {ex}')

    def destroy_node(self):
        self.subscription.destroy()
        self.transformed_pose_publisher.destroy()
        self.marker_publisher.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    cube_pose_transformer = CubePoseTransformer()

    try:
        rclpy.spin(cube_pose_transformer)
    except KeyboardInterrupt:
        pass
    finally:
        cube_pose_transformer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
