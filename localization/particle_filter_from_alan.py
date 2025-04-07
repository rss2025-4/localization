from typing import Callable

import matplotlib.pyplot as plt
import numpy as np
import tf2_ros
from liblocalization import deterministic_motion_tracker
from liblocalization.api import LocalizationBase, localization_params
from liblocalization.controllers.particles import particles_model, particles_params
from nav_msgs.msg import OccupancyGrid, Odometry
from odom_transformer.transformer import Transformer
from rclpy.node import Node
from rclpy.time import Time
from scan_simulator_2d import PyScanSimulator2D
from sensor_msgs.msg import LaserScan
from tf2_ros import (
    Node,
    PoseWithCovarianceStamped,
    TransformStamped,
    rclpy,
)
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

from localization.helpers import PoseTransformTools

# from .api_alan import LocalizationBase, localization_params


class ExampleSimNode(Node):
    def __init__(
        self, controller_init: Callable[[localization_params], LocalizationBase]
    ):
        super().__init__("ExampleSimNode")
        self.pose_transform_tools = PoseTransformTools()

        self.num_beams_per_particle = 100
        self.scan_theta_discretization = 500.0
        self.scan_field_of_view = 4.71
        # self.num_beams_per_particle = (
        #     self.get_parameter("num_beams_per_particle")
        #     .get_parameter_value()
        #     .integer_value
        # )
        # self.scan_theta_discretization = (
        #     self.get_parameter("scan_theta_discretization")
        #     .get_parameter_value()
        #     .double_value
        # )
        # self.scan_field_of_view = (
        #     self.get_parameter("scan_field_of_view").get_parameter_value().double_value
        # )

        self.controller_init = controller_init
        self.controller = None

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 1
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/vesc/odom", self.odom_callback, 1
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 1
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 1
        )

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)
        self.transform_pub = self.create_publisher(TransformStamped, "/base_link_pf", 1)

        self.visualization_pub = self.create_publisher(Marker, "/visualization", 10)

        # Added for plotting laser scan

        self.actual_x = []
        self.actual_y = []

        self.estimated_x = []
        self.estimated_y = []

        self.initial_time = self.get_clock().now()
        # self.create_timer(0.1, self.check_duration)

        self.estimated_pose = None

        # scan simulator
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization,
        )

        self.odom_transformer = None

    def get_controller(self) -> LocalizationBase | None:
        if self.controller is None:
            return None
        if self.odom_transformer is None:
            try:
                self.odom_transformer = Transformer(
                    self.tfBuffer.lookup_transform(
                        "laser", "base_link", Time()
                    ).transform
                )
            except Exception as e:
                print("failed to get transform:", e)
                return

        return self.controller

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if controller := self.get_controller():
            assert self.odom_transformer is not None
            pos_laser = self.odom_transformer.transform_pose(msg.pose)

            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = "laser"

            pos = pos_laser.pose.position
            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = pos.z

            t.transform.rotation = pos_laser.pose.orientation

            controller.set_pose(t)

    def map_callback(self, map_msg: OccupancyGrid):
        self.controller = self.controller_init(
            localization_params(
                map=map_msg,
                n_laser_points=1081,  # added for real racecar
                map_frame="map",
                marker_callback=self.marker_callback,
                ground_truth_callback=self.ground_truth_callback,
            )
        )
        assert isinstance(self.controller, LocalizationBase)

        # For scan sim
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.0
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = euler_from_quaternion(
            (origin_o.x, origin_o.y, origin_o.z, origin_o.w)
        )
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5,
        )  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")

    def odom_callback(self, msg: Odometry):
        if controller := self.get_controller():
            assert self.odom_transformer is not None

            # for real racecar, need to negate?
            msg.header.frame_id = "map"  # cause needs to be map

            assert msg.twist.twist.linear.y == 0.0
            msg.twist.twist.linear.x = -msg.twist.twist.linear.x
            msg.twist.twist.angular.z = -msg.twist.twist.angular.z

            assert msg.child_frame_id == "base_link"
            odom = Odometry(header=msg.header, child_frame_id="laser")
            # odom = Odometry(header="map", child_frame_id="laser")
            odom.pose = self.odom_transformer.transform_pose(msg.pose)
            odom.twist = self.odom_transformer.transform_twist(msg.twist)

            controller.odom_callback(odom)
            # print("pose:", controller.get_pose())
            # print("particles:", controller.get_particles())
            self.publish_estimated_pose()

    def lidar_callback(self, msg: LaserScan):
        if controller := self.get_controller():
            controller.lidar_callback(msg)
            self.assign_lidar_ranges(msg)
            print("laser callback")
            self.publish_estimated_pose()

    def check_duration(self):
        pass

        # if (self.get_clock().now() - self.initial_time).nanoseconds * 1e-9 > 40.0:
        #     print(" in duration check")
        #     plt.plot(self.actual_x, self.actual_y, "ro")

        #     particle = np.array(self.estimated_pose).reshape(-1, 3)
        #     # particle = np.array([0.0,0.0,0.0]).reshape(-1,3)
        #     print("particle", particle)
        #     scan_from_particle = self.scan_sim.scan(particle)
        #     print("scan_from_particle", scan_from_particle)
        #     self.assign_estimated_lidar_ranges(scan_from_particle, particle)
        #     # print(self.estimated_x, self.estimated_y)
        #     plt.plot(self.estimated_x, self.estimated_y, "bo")
        #     plt.savefig(f"laser_scan_3.png")
        #     plt.close()
        #     print("finished plotting")
        #     rclpy.shutdown()
        #     print("closed node")
        # else:
        #     print(
        #         "not time yet: ",
        #         (self.get_clock().now() - self.initial_time).nanoseconds * 1e-9,
        #     )

    def marker_callback(self, marker: Marker):
        self.visualization_pub.publish(marker)

    def ground_truth_callback(self) -> TransformStamped | None:
        try:
            t = self.tfBuffer.lookup_transform("map", "laser", Time())
        except Exception as e:
            print("failed to get transform:", e)
            return None
        return t

    def publish_estimated_pose(self):
        if controller := self.get_controller():
            transform_msg = controller.get_pose()
            transform_msg.child_frame_id = "base_link"
            # print("pose", transform_msg)
            x_avg = transform_msg.transform.translation.x
            y_avg = transform_msg.transform.translation.y
            theta_avg = transform_msg.transform.rotation.z
            print(
                "pose",
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y,
            )

            odom_msg = Odometry()
            odom_msg.header.frame_id = "map"
            odom_msg.child_frame_id = "laser"
            odom_msg.pose.pose.position.x = x_avg
            odom_msg.pose.pose.position.y = y_avg
            odom_msg.pose.pose.orientation.z = theta_avg

            pose = np.array([x_avg, y_avg, theta_avg])
            # print("pose estimate", x_avg, y_avg, theta_avg)
            self.estimated_pose = pose

            self.odom_pub.publish(odom_msg)  # return controller.get_pose()

            self.transform_pub.publish(transform_msg)
        else:
            raise Exception("Controller not initialized")

    def assign_lidar_ranges(self, scan_msg: LaserScan):
        try:
            t = self.tfBuffer.lookup_transform(
                "map", "laser", Time()
            )  # on racecar, laser_model
        except Exception as e:
            print("failed to get transform:", e)
            return None
        # transform all laser points from laser frame to map frame
        pose_laser_map = [
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.rotation.z,
        ]
        poses_laser_map = np.vstack([pose_laser_map] * len(scan_msg.ranges))
        # print("pose_laser_map", poses_laser_map)
        # 0. Get scan_msg data
        ranges = np.array(scan_msg.ranges, dtype="float32")
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment

        # 1. Slice up the scan
        angles = np.arange(angle_min, angle_max, angle_increment)

        # 1a. Get x,y position of each point relative to robot laser frame
        # we know mapping from ranges to angle
        x_array = ranges * np.cos(angles)  # if robot's forward is pos x
        y_array = ranges * np.sin(angles)

        print("lidar ranges", x_array, y_array, angles)
        poses_range_laser = np.column_stack((x_array, y_array, angles))
        # print("poses_range_laser", poses_range_laser)

        pose_range_map = self.pose_transform_tools.get_new_poses(
            poses_laser_map, poses_range_laser
        )

        self.actual_x = pose_range_map[:, 0]
        self.actual_y = pose_range_map[:, 1]

    def assign_estimated_lidar_ranges(self, scan_ranges, particle):
        # try:
        #     t = self.tfBuffer.lookup_transform("map", "laser", Time())
        # except Exception as e:
        #     print("failed to get transform:", e)
        #     return None

        # # Transformation from laser frame to map frame
        # pose_laser_map = [t.transform.translation.x, t.transform.translation.y, t.transform.rotation.z]

        # print("lidar pose", pose_laser_map)
        # poses_laser_map = np.vstack([pose_laser_map] * len(scan_ranges))
        poses_laser_map = np.vstack(
            [particle] * len(scan_ranges)
        )  # estimated particle is of laser to map
        # print("pose_laser_map", poses_laser_map)
        print("shape", np.shape(scan_ranges))

        # 0. Get scan_msg data
        ranges = scan_ranges.ravel()  # np.array(scan_ranges, dtype="float32")
        angle_min = 0 - self.scan_field_of_view / 2.0  # scan_msg.angle_min
        angle_max = 0 + self.scan_field_of_view / 2.0  # scan_msg.angle_max
        angle_increment = (
            self.scan_field_of_view / np.shape(scan_ranges)[1]
        )  # self.scan_theta_discretization #scan_msg.angle_increment
        print("angle_increment", angle_increment)
        # 1. Slice up the scan
        angles = np.arange(angle_min, angle_max, angle_increment)

        # 1a. Get x,y position of each point relative to robot laser frame
        # we know mapping from ranges to angle
        x_array = ranges * np.cos(angles)  # if robot's forward is pos x
        y_array = ranges * np.sin(angles)

        print("estimated lidar ranges", x_array, y_array, angles)
        poses_range_laser = np.column_stack((x_array, y_array, angles))
        # print("poses_range_laser", poses_range_laser)
        pose_range_map = self.pose_transform_tools.get_new_poses(
            poses_laser_map, poses_range_laser
        )

        self.estimated_x = poses_range_laser[:, 0]  # - particle[0][0]
        self.estimated_y = poses_range_laser[:, 1]  # + particle[0][1]


def examplemain():
    """examle (deterministic_motion_tracker)"""
    rclpy.init()
    rclpy.spin(ExampleSimNode(deterministic_motion_tracker))
    assert False, "unreachable"


def examplemain2():
    """examle (particles_model)"""
    rclpy.init()
    rclpy.spin(ExampleSimNode(particles_model(particles_params(n_particles=200))))
    assert False, "unreachable"


if __name__ == "__main__":
    # examplemain()
    print("hi")
    examplemain2()
