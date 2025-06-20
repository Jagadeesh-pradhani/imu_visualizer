import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion
import numpy as np


class ImuVisualizer(Node):

    def __init__(self):
        super().__init__('imu_visualizer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Marker, '/imu_visualization', 10)
        self.marker = Marker()
        self.marker.header.frame_id = "imu_link"  # Set the frame ID
        self.marker.ns = "imu_visualizer"
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

    def imu_callback(self, msg):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation = msg.orientation

        # Calculate the end point of the arrow based on the acceleration vector
        acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        acceleration_magnitude = np.linalg.norm(acceleration)

        if acceleration_magnitude > 0:
            normalized_acceleration = acceleration / acceleration_magnitude
            end_point = Point()
            end_point.x = normalized_acceleration[0] * self.marker.scale.x
            end_point.y = normalized_acceleration[1] * self.marker.scale.x
            end_point.z = normalized_acceleration[2] * self.marker.scale.x

            self.marker.points = []
            self.marker.points.append(Point(x=0.0, y=0.0, z=0.0))
            self.marker.points.append(end_point)
        else:
            # If acceleration is zero, draw a small arrow in a default direction
            end_point = Point()
            end_point.x = 0.1
            end_point.y = 0.0
            end_point.z = 0.0

            self.marker.points = []
            self.marker.points.append(Point(x=0.0, y=0.0, z=0.0))
            self.marker.points.append(end_point)

        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    imu_visualizer = ImuVisualizer()
    rclpy.spin(imu_visualizer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
