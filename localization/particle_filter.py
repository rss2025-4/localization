from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.node import Node
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
        

        # visualize motion model particles
        # self.particles_pub = self.create_publisher(Marker, "/motion_model_particles", 1)
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
    def laser_callback(self, msg):
        observation = msg.ranges
        self.sensor_model.evaluate(updated_particles, observation)
        # pass
        # self.sensor_model.update_sensor(msg)
    def odom_callback(self, msg):
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        current_time = msg.header.stamp*1e-9
        
        updated_particles = self.motion_model.update_odometry(x_velocity, y_velocity, angular_velocity, current_time)
        # pass
    # def publish_particles(self):
    #     all_points = Marker()
    #     all_points.type = Marker.POINTS
    #     all_points.header.frame_id = "/laser"

    #     all_points.scale.x = 0.1
    #     all_points.scale.y = 0.1
    #     all_points.color.a = 1.
    #     all_points.color.r = color[0]
    #     all_points.color.g = color[1]
    #     all_points.color.g = color[2]
        
    #     updated_particles = [[0.0,1.0,0.0],[0.0,2.0,0.0]]#self.motion_model.evaluate()

    #     for particle in updated_particles:
    #         p = Point()
    #         p.x = particle[0]
    #         p.y = particle[1]
    #         # p.z = 0
    #         all_points.points.append(p)

    #     self.particles_pub.publish(all_points)
    def pose_callback(self, msg):
        pass
def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
