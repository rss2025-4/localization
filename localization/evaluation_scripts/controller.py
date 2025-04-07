from typing import Callable

import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from odom_transformer.transformer import Transformer
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import (
    Node,
    PoseWithCovarianceStamped,
    TransformStamped,
    rclpy,
)
from visualization_msgs.msg import Marker

from liblocalization import deterministic_motion_tracker
from liblocalization.api import LocalizationBase, localization_params
from liblocalization.controllers.particles import particles_model, particles_params
from geometry_msgs.msg import Point



class ExampleSimNode(Node):
    def __init__(
        self, controller_init: Callable[[localization_params], LocalizationBase]
    ):
        super().__init__("ExampleSimNode")

        self.controller_init = controller_init
        self.controller = None

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 1
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/noisy_odom", self.odom_callback, 1
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 1
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 1
        )
        # self.pose_estimate_sub = self.create_subscription(Odometry, "/pf/pose/odom",
        #                                          self.pose_estimate_callback,
        #                                          1)        
        self.particle_estimate_publisher = self.create_publisher(Marker, "/particle_estimate", 1)

        self.visualization_pub = self.create_publisher(Marker, "/visualization", 10)
        self.debug = False
        self.did_set_pose = False
    def pose_estimate_callback(self, particles):
        """
        eghosa's pose callback to display particles but on estimate
        """
        particle_pts = Marker()
        particle_pts.type = Marker.POINTS
        
        particle_pts.header.frame_id = "/map"
        
        particle_pts.scale.x = 0.1
        particle_pts.scale.y = 0.1
        particle_pts.color.a = 1.
        particle_pts.color.r, particle_pts.color.b, particle_pts.color.g = 0.482, 0.431, 0.922
        
        for particle in particles:
            if self.debug:
                print("particle", particle)
                print("particle x", particle[0])
            p = Point()
            p.x = float(particle[0])
            p.y = float(particle[1])
            p.z = float(particle[2])
            particle_pts.points.append(p)
        self.particle_estimate_publisher.publish(particle_pts)
        self.get_logger().info("published particle estimate")
        
    def get_controller(self) -> LocalizationBase | None:
        if self.controller is None:
            return None
        if not self.did_set_pose:
            try:
                t = self.tfBuffer.lookup_transform("map", "laser", Time())
            except Exception as e:
                print("failed to get transform:", e)
                return
            self.odom_transformer = Transformer(
                self.tfBuffer.lookup_transform("laser", "base_link", Time()).transform
            )
            self.controller.set_pose(t)
            self.did_set_pose = True

        return self.controller

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self.controller is not None:
            pos_laser = self.odom_transformer.transform_pose(msg.pose)

            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = "laser"

            pos = pos_laser.pose.position
            t.transform.translation.x = pos.x
            t.transform.translation.y = pos.y
            t.transform.translation.z = pos.z

            t.transform.rotation = pos_laser.pose.orientation

            self.controller.set_pose(t)
            
            particles = self.controller.get_particles()
            print("i got particles")
            self.pose_estimate_callback(particles)
        else:
            print("controller not initialized yet")
            

    def map_callback(self, map_msg: OccupancyGrid):
        self.controller = self.controller_init(
            localization_params(
                map=map_msg,
                marker_callback=self.marker_callback,
                ground_truth_callback=self.ground_truth_callback,
            )
        )
        assert isinstance(self.controller, LocalizationBase)

    def odom_callback(self, msg: Odometry):
        if controller := self.get_controller():

            assert msg.child_frame_id == "base_link"
            odom = Odometry(header=msg.header, child_frame_id="laser")
            odom.pose = self.odom_transformer.transform_pose(msg.pose)
            odom.twist = self.odom_transformer.transform_twist(msg.twist)

            controller.odom_callback(odom)
            particles = controller.get_particles()
            self.pose_estimate_callback(particles)
            # print("pose:", controller.get_pose())
            # print("particles:", controller.get_particles())

    def lidar_callback(self, msg: LaserScan):
        if controller := self.get_controller():
            controller.lidar_callback(msg)

    def marker_callback(self, marker: Marker):
        self.visualization_pub.publish(marker)

    def ground_truth_callback(self) -> TransformStamped | None:
        try:
            t = self.tfBuffer.lookup_transform("map", "laser", Time())
        except Exception as e:
            print("failed to get transform:", e)
            return None
        return t


# def examplemain():
#     """examle (deterministic_motion_tracker)"""
#     rclpy.init()
#     rclpy.spin(ExampleSimNode(deterministic_motion_tracker))
#     assert False, "unreachable"


def examplemain2():
    """examle (particles_model)"""
    rclpy.init()
    rclpy.spin(ExampleSimNode(particles_model(particles_params(n_particles=200))))
    assert False, "unreachable"
if __name__ == "__main__":
    examplemain2()

# import abc
# from abc import abstractmethod
# from dataclasses import dataclass
# from typing import Callable

# import tf2_ros
# from nav_msgs.msg import OccupancyGrid, Odometry
# from odom_transformer.transformer import Transformer
# from rclpy.node import Node
# from rclpy.time import Time
# from sensor_msgs.msg import LaserScan
# from tf2_ros import (
#     Node,
#     TransformStamped,
#     rclpy,
# )
# from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
# from visualization_msgs.msg import Marker
# from liblocalization import LocalizationBase
# from liblocalization import localization_params
# from geometry_msgs.msg import Point
# # from custom_msgs.msg import Particle


# class ExampleSimNode(Node):
#     def __init__(
#         self, controller_init: Callable[[localization_params], LocalizationBase]
#     ):
#         super().__init__("ExampleSimNode")

#         self.controller_init = controller_init
#         self.controller = None

#         self.tfBuffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)

#         self.map_sub = self.create_subscription(
#             OccupancyGrid, "/map", self.map_callback, 1
#         )
#         self.odom_sub = self.create_subscription(
#             Odometry, "/odom", self.odom_callback, 1
#         )
#         self.lidar_sub = self.create_subscription(
#             LaserScan, "/scan", self.lidar_callback, 1
#         )
#         self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
#                                                  self.pose_estimate_callback,
#                                                  1)
#         self.pose_estimate_sub = self.create_subscription(Odometry, "/pf/pose/odom",
#                                                  self.pose_estimate_callback,
#                                                  1)           
#         self.visualization_pub = self.create_publisher(Marker, "/visualization", 10)
#         self.particle_estimate_publisher = self.create_publisher(Marker, "/particle_estimate", 1)
#         self.did_set_pose = False

#     def get_controller(self) -> LocalizationBase | None:
#         if self.controller is None:
#             return None
#         if not self.did_set_pose:
#             try:
#                 t = self.tfBuffer.lookup_transform("map", "laser", Time())
#             except Exception as e:
#                 print("failed to get transform:", e)
#                 return
#             self.odom_transformer = Transformer(
#                 self.tfBuffer.lookup_transform("laser", "base_link", Time()).transform
#             )
#             self.controller.set_pose(t)
#             self.did_set_pose = True
#             print("set pose:")

#         return self.controller
#     def pose_estimate_callback(self, msg):
#         """
#         eghosa's pose callback to display particles but on estimate
#         """
#         print("pose_estimate_callback")
#         print(f'received pose estimate: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}')
#         if controller := self.get_controller():
#             particles = controller.get_particles()
#             particle_pts = Marker()
#             particle_pts.type = Marker.POINTS
            
#             particle_pts.header.frame_id = "/map"
            
#             particle_pts.scale.x = 0.1
#             particle_pts.scale.y = 0.1
#             particle_pts.scale.z = 1
#             particle_pts.color.a = 1.
#             particle_pts.color.r, particle_pts.color.b, particle_pts.color.g = 0.482, 0.431, 0.922
            
#             for particle in particles:
#                 p = Point()
#                 p.x = particle[0]
#                 p.y = particle[1]
#                 p.z = particle[2]
#                 particle_pts.points.append(p)
#             self.particle_estimate_publisher.publish(particle_pts)
#             self.get_logger().info("published particle estimate")
#         # print("first particle", self.particles[0])    
#         else:
#             print("controller not initialized yet")
#     def map_callback(self, map_msg: OccupancyGrid):
#         print("map_callback")
#         self.controller = self.controller_init(
#             localization_params(
#                 map=map_msg,
#                 marker_callback=self.marker_callback,
#                 ground_truth_callback=self.ground_truth_callback,
#             )
#         )
#         assert isinstance(self.controller, LocalizationBase)

#     def odom_callback(self, msg: Odometry):
#         print("odom_callback")
#         if controller := self.get_controller():

#             assert msg.child_frame_id == "base_link"
#             odom = Odometry(header=msg.header, child_frame_id="laser")
#             odom.pose = self.odom_transformer.transform_pose(msg.pose)
#             odom.twist = self.odom_transformer.transform_twist(msg.twist)

#             controller.odom_callback(odom)
#             # print("pose:", controller.get_pose())

#     def lidar_callback(self, msg: LaserScan):
#         print("lidar_callback") 
#         if controller := self.get_controller():
#             controller.lidar_callback(msg)

#     def marker_callback(self, marker: Marker):
#         self.visualization_pub.publish(marker)

#     def ground_truth_callback(self) -> TransformStamped | None:
#         try:
#             t = self.tfBuffer.lookup_transform("map", "laser", Time())
#         except Exception as e:
#             print("failed to get transform:", e)
#             return None
#         return t


# def examplemain():
#     # deterministic_motion_tracker = just motion model mode 
#     # p
#     from liblocalization import  deterministic_motion_tracker

#     rclpy.init()
#     rclpy.spin(ExampleSimNode(deterministic_motion_tracker))
#     assert False, "unreachable"
    
# if __name__ == "__main__":
#     examplemain()
    
    

