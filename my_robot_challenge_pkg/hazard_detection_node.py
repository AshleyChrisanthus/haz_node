#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from find_object_2d.msg import ObjectsStamped
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import PointStamped
import tf2_ros
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class HazardDetectionNode(Node):
    def __init__(self):
        super().__init__('hazard_detection_node')

        # Subscribe to object detections
        self.create_subscription(ObjectsStamped, '/objectsStamped', self.handle_objects, 10)
        # self.get_logger().info('Subscribed to /objectsStamped')

        # Subscribe to laser scan data
        self.create_subscription(LaserScan, '/scan', self.store_scan, 10)
        # self.get_logger().info('Subscribed to /scan')

        # Publisher for markers (for visualization in RViz)
        self.marker_publisher = self.create_publisher(Marker, '/hazards', 10)
        # self.get_logger().info('Publisher set up for /hazards')

        # TF Buffer and Listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # self.get_logger().info('TF listener initialized')

        # Initialize instance variables
        self.laser_scan = None
        self.published_ids = set()

        self.get_logger().info('Hazard Detection Node ready.')

    def store_scan(self, msg):
        self.laser_scan = msg
        # self.get_logger().info(f'Laser scan data received: {len(msg.ranges)} range readings')

    def handle_objects(self, msg):
        if self.laser_scan is None or not msg.objects.data:
            self.get_logger().warning('Missing necessary data: laser_scan or object detections.')
            return

        if len(msg.objects.data) % 12 != 0:
            self.get_logger().error("Invalid ObjectsStamped data length")
            return

        self.get_logger().info(f'Handling {len(msg.objects.data) // 12} objects.')

        for i in range(0, len(msg.objects.data), 12):
            data = msg.objects.data[i:i+12]
            obj_id = int(data[0])
            x_min, y_min = data[3], data[4]
            width, height = data[5], data[6]

            self.get_logger().info(f'Object data: {str(data)}')

            # Map object centroid to laser scan angle
            image_width = 800  # Adjust based on your object detection camera's resolution
            u = x_min + width / 2.0
            fov_ratio = u / image_width  # Normalize to [0, 1]
            angle = self.laser_scan.angle_min + fov_ratio * (self.laser_scan.angle_max - self.laser_scan.angle_min)
            index = int((angle - self.laser_scan.angle_min) / self.laser_scan.angle_increment)

            if 0 <= index < len(self.laser_scan.ranges):
                depth = self.laser_scan.ranges[index]
                self.get_logger().info(f'Laser depth for object {obj_id}: {depth:.2f} meters')
                if not np.isfinite(depth) or depth < self.laser_scan.range_min or depth > self.laser_scan.range_max:
                    self.get_logger().warning(f"Invalid depth {depth} at index {index} for object {obj_id}")
                    continue

                # Convert depth and angle to a 3D point in the laser frame
                # Assuming 2D laser scan (z=0)
                point_laser = PointStamped()
                point_laser.header.frame_id = self.laser_scan.header.frame_id
                point_laser.header.stamp = self.laser_scan.header.stamp
                point_laser.point.x = depth * np.cos(angle)
                point_laser.point.y = depth * np.sin(angle)
                point_laser.point.z = 0.0

                try:
                    # Check if transform is available
                    if self.tf_buffer.can_transform('map', self.laser_scan.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)):
                        # Transform the point to the map frame using the latest available transform
                        point_map = self.tf_buffer.transform(point_laser, 'map', timeout=rclpy.duration.Duration(seconds=0.1))
                        self.get_logger().info(f"Transformed point to map frame: ({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})")
                        # Publish the marker at the transformed point
                        self.publish_marker(obj_id, point_map)
                    else:
                        self.get_logger().warning(f"Transform from {self.laser_scan.header.frame_id} to map not available yet.")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().error(f"TF transform error: {str(e)}")
                    continue

    def publish_marker(self, obj_id, point):
        if obj_id in self.published_ids:
            self.get_logger().info(f"Hazard {obj_id} already published.")
            return

        marker = Marker()
        marker.header.frame_id = 'map'
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
        marker.lifetime = rclpy.duration.Duration(seconds=10.0).to_msg()

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
