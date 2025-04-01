from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion

from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import rclpy

assert rclpy


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        
        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('scan_topic', "/scan")

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.NUM_PARTICLES = 200
        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)
        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)

        self.get_logger().info("=============+READY+=============")

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        
        # odom data = motion model 
        # sensor model to resample 
        self.particles = None
        self.particle_publisher = self.create_publisher(Marker, "/particle", 1)
        
        
    def laser_callback(self, msg):
        self.sensor_model.laser_callback(msg)
    def odom_callback(self, msg):
        if not self.particles:
            return
        self.particles = self.motion_model.evaluate(self.particles, msg)
    def pose_callback(self, msg):
        
        std_x = 0.5
        std_y = 0.5
        std_theta = 0.25
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])
        xs = np.random.normal(x, std_x, self.NUM_PARTICLES)
        ys = np.random.normal(y, std_y, self.NUM_PARTICLES)
        thetas = np.random.normal(yaw, std_theta, self.NUM_PARTICLES)
        self.particles = np.vstack((xs, ys, thetas)).T
        particle_pts = Marker()
        particle_pts.type = Marker.POINTS
        
        particle_pts.header.frame_id = "/map"
        
        particle_pts.scale.x = 0.1
        particle_pts.scale.y = 0.1
        particle_pts.color.a = 1.
        particle_pts.color.r, particle_pts.color.b, particle_pts.color.g = 1.0, 0.69, 0.651
        
        for particle in self.particles:
            p = Point()
            p.x = particle[0]
            p.y = particle[1]
            particle_pts.points.append(p)
        self.particle_publisher.publish(particle_pts)
        


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
