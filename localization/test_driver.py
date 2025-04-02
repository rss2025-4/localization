#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
# from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

# from wall_follower.visualization_tools import VisualizationTools


class Driver(Node):

    def __init__(self):
        super().__init__("driver")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS!
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter(
            "drive_topic", "/drive"
        )  # /vesc/low_level/input/navigation #drive
        # self.declare_parameter("side", "default")
        self.declare_parameter("velocity", 1.0)  # "default"
        # self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.DRIVE_TOPIC = (
            self.get_parameter("drive_topic").get_parameter_value().string_value
        )
        # self.SIDE = self.get_parameter("side").get_parameter_value().integer_value

        self.VELOCITY = (
            self.get_parameter("velocity").get_parameter_value().double_value
        )
        # self.DESIRED_DISTANCE = (
        #     self.get_parameter("desired_distance").get_parameter_value().double_value
        # )

        print("scan topic: ", self.SCAN_TOPIC)
        print("drive topic: ", self.DRIVE_TOPIC)
        print("velocity: ", self.VELOCITY)

        self.add_on_set_parameters_callback(self.parameters_callback)
        # TODO: Initialize your publishers and subscribers here

        # Drive publisher
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.DRIVE_TOPIC, 10
        )

        # FOR SAFETY CONTROLLER TEST
        drive_timer_period = 1 / 60  # seconds, 60 Hz
        self.drive_timer = self.create_timer(drive_timer_period, self.drive_callback)

        self.alert_subscriber = self.create_subscription(
            String, "/alert", self.alert_callback, 10
        )

        self.stop = False

    # TODO: Write your callback functions here
    def alert_callback(self, alert_msg):
        """ For simulation only, but can use as for debug """
        if alert_msg.data == "STOP":
            self.stop = True
            self.get_logger().info('Got alert msg: "%s"' % alert_msg.data)
        else:
            self.stop = False

    def drive_callback(self):
        # declare msg
        drive_msg = AckermannDriveStamped()

        curr_time = self.get_clock().now()

        # assign values
        drive_msg.header.stamp = curr_time.to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = 1.0  # rad
        drive_msg.drive.steering_angle_velocity = 0.0  # rad/s
        drive_msg.drive.speed = self.VELOCITY  # m/s
        drive_msg.drive.acceleration = 0.0  # m/s^2
        drive_msg.drive.jerk = 0.0  # m/s^3

        # publish message
        if not self.stop:
            self.drive_publisher.publish(drive_msg)
            self.get_logger().info('Published drive msg: "%s"' % drive_msg.drive.speed)

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!

        This is used by the test cases to modify the parameters during testing.
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == "side":
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == "velocity":
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == "desired_distance":
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(
                    f"Updated desired_distance to {self.DESIRED_DISTANCE}"
                )
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
