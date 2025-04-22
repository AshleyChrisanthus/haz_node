#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from find_object_2d.msg import ObjectsStamped
from sensor_msgs.msg import LaserScan, CameraInfo
from tf2_geometry_msgs import PointStamped
import tf2_ros
import numpy as np

from visualization_msgs.msg import Marker

class HazardDetectionNode(Node):
    def __init__(self):
        super().__init__('hazard_detection_node')

        self.create_subscription(ObjectsStamped, '/objectsStamped', self.handle_objects, 10)
        self.create_subscription(LaserScan, '/scan', self.store_scan, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.store_camera_info, 10)

        self.marker_publisher = self.create_publisher(Marker, '/hazards', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.laser_scan = None
        self.camera_info = None
        self.published_ids = set()

        self.get_logger().info('Hazard Detection Node ready.')

    def store_camera_info(self, msg):
        self.camera_info = msg

    def store_scan(self, msg):
        self.laser_scan = msg

    def handle_objects(self, msg):
        if self.laser_scan is None or self.camera_info is None or not msg.objects.data:
            return

        for i in range(0, len(msg.objects.data), 12):
            data = msg.objects.data[i:i+12]
            obj_id = int(data[0])
            x_min, y_min = data[3], data[4]
            width, height = data[5], data[6]

            u = x_min + width / 2.0
            v = y_min + height / 2.0

            fx, fy = self.camera_info.k[0], self.camera_info.k[4]
            cx, cy = self.camera_info.k[2], self.camera_info.k[5]

            angle = -np.arctan2(u - cx, fx)
            index = int((angle - self.laser_scan.angle_min) / self.laser_scan.angle_increment)

            if 0 <= index < len(self.laser_scan.ranges):
                depth = self.laser_scan.ranges[index]
                if not np.isfinite(depth):
                    continue

                # X = depth * (u - cx) / fx
                # Y = depth * (v - cy) / fy
                # Z = depth

                angle = self.laser_scan.angle_min + index * self.laser_scan.angle_increment
                X = depth * np.cos(angle)
                Y = depth * np.sin(angle)
                Z = 0.0

                point = PointStamped()
                # point.header.frame_id = self.camera_info.header.frame_id
                point.header.frame_id = self.laser_scan.header.frame_id
                # point.header.stamp = self.get_clock().now().to_msg()
                point.header.stamp = self.laser_scan.header.stamp
                # point.header.stamp = self.camera_info.header.stamp
                # latest_common_time = self.tf_buffer.get_latest_common_time('camera_depth_optical_frame', 'map')
                # point.header.stamp = latest_common_time

                point.point.x = X
                point.point.y = Y
                point.point.z = Z

                try:
                    transformed = self.tf_buffer.transform(point, 'base_link', timeout=rclpy.duration.Duration(seconds=0.1))
                    
                    # # Now transform from base_link to map
                    transformed = self.tf_buffer.transform(transformed, 'map', timeout=rclpy.duration.Duration(seconds=0.1))

                    self.publish_marker(obj_id, transformed)

                except Exception as e:
                    self.get_logger().error(f"Transform failed: {str(e)}")

    def publish_marker(self, obj_id, point):
        if obj_id in self.published_ids:
            self.get_logger().info(f"Hazard {obj_id} already published.")
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        # marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hazards'
        marker.id = obj_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.marker_publisher.publish(marker)
        self.published_ids.add(obj_id)
        self.get_logger().info(f"Published marker for hazard {obj_id} at ({point.point.x:.2f}, {point.point.y:.2f}, {point.point.z:.2f})")

def main():
    rclpy.init()
    node = HazardDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
