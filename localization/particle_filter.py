from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from geometry_msgs.msg import Point


from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import rclpy

assert rclpy

import numpy as np

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

        self.laser_callback_freq = 20 #Hz
        self.laser_callback_prev_time = 0.0

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
        self.pose_estimate_sub = self.create_subscription(Odometry, "/pf/pose/odom",
                                                 self.pose_estimate_callback,
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
    
        # from eghosa's code: 
        self.particles = np.zeros((self.NUM_PARTICLES, 3))
        self.particle_publisher = self.create_publisher(Marker, "/particle", 1)
        self.particle_estimate_publisher = self.create_publisher(Marker, "/particle_estimate", 1)
        self.particle_probabilites = np.ones(self.NUM_PARTICLES, dtype=float)
        
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        self.tf_broadcaster = TransformBroadcaster(self)

       
    def laser_callback(self, msg):
        # current_time_msg = self.get_clock().now()
        # seconds, nanoseconds = current_time_msg.seconds_nanoseconds()
        # current_time = seconds + nanoseconds * 1e-9
        # if current_time - self.laser_callback_prev_time < 1.0/self.laser_callback_freq:
        #     return
        # pass
        # self.laser_callback_prev_time = current_time
        observation = msg.ranges
        self.particle_probabilites = self.sensor_model.evaluate(self.particles, observation)
        
        self.particles = self.resample(self.particle_probabilites, self.particles)
        # if not self.particle_probabilites:
        #     print("fixed")
        #     self.particle_probabilities = np.ones(self.NUM_PARTICLES, dtype=float)
        # get array of most probabilities
        self.get_pose(self.particles) # publishes pose estimate
        self.get_logger().info("laser callback")

    def odom_callback(self, msg):
        # pass
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        current_time_msg = self.get_clock().now()
        seconds, nanoseconds = current_time_msg.seconds_nanoseconds()
        current_time = seconds + nanoseconds * 1e-9

        odometry = self.motion_model.update_odometry(x_velocity, y_velocity, angular_velocity, current_time)
        self.particles = self.motion_model.evaluate(self.particles, odometry)
        
        self.get_pose(self.particles) # publishes pose estimate
        self.get_logger().info("odom callback")
    
    def resample(self, particle_probabilities, particles):
        if particle_probabilities is None:
            self.get_logger().warn("NOT resampled")
            return self.particles

        # Create list of options to sample from
        # self.get_logger().info("particle_probabilities_size %s" % len(particle_probabilities))
        # self.get_logger().info("particles_size %s" % len(particles))
        
        
        # particle_probabilities = np.nan_to_num(particle_probabilities, nan=0)
        particle_probabilities = particle_probabilities / np.sum(particle_probabilities)
        particle_probabilities = np.nan_to_num(particle_probabilities, nan=0)

        options = np.arange(0,len(particle_probabilities),1)

        # self.get_logger().info("hi %s" % particle_probabilities)

        # if np.sum(particle_probabilities) <= 1.0:
        #     return particles
            
        selections = np.random.choice(options, len(particles), p=particle_probabilities)
        
        self.get_logger().info("resampled")
        return particles[selections]
       
    def get_pose(self, particles):
        """
        Calculate and return the average pose (x, y, theta) of the particles.

        The average x and y positions are calculated as the arithmetic mean,
        and the average orientation (theta) is calculated using the circular mean.

        Args:
        particles (list or np.ndarray): List of particles, where each particle is [x, y, theta]

        Returns:
        np.ndarray: The estimated pose as [x_avg, y_avg, theta_avg]
        """

        # Convert particles to a numpy array for easier manipulation
        # particles = np.array(self.particles)

        # Calculate the average x and y positions
        x_avg = np.average(particles[:, 0], weights = self.particle_probabilites)
        y_avg = np.average(particles[:, 1], weights = self.particle_probabilites)

        # Circular mean for theta (orientation)
        theta = particles[:, 2]
        # if not self.particle_probabilites:
        #     print("warning doesnt exist")
        sin_sum = np.sum(np.sin(theta))
        cos_sum = np.sum( np.cos(theta))
        # sin_sum = np.sum(self.particle_probabilites * np.sin(theta))
        # cos_sum = np.sum(self.particle_probabilites * np.cos(theta))
        
        theta_avg = np.arctan2(sin_sum, cos_sum)  # Circular mean to get average angle
        pose = [x_avg, y_avg, theta_avg]
        # publish pose 
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = x_avg
        odom_msg.pose.pose.position.y = y_avg
        odom_msg.pose.pose.orientation.z = theta_avg

        pose = [x_avg,y_avg,theta_avg]
        # print("pose estimate", x_avg, y_avg, theta_avg)

        self.odom_pub.publish(odom_msg)
        # self.get_logger().info("Published pose estimate")

        self.publish_transform(pose)
        self.get_logger().info("Published transform pf to map")

    def publish_transform(self,pose):
        [x,y,theta] = pose
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()


        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x #msg.pose.pose.position.x
        t.transform.translation.y = y #msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = quaternion_from_euler(0, 0, msg.pose.pose.orientation.z)
        t.transform.rotation.x = 0.0 #q[0]
        t.transform.rotation.y = 0.0 #q[1]
        t.transform.rotation.z = np.sin(theta/2) #q[2]
        t.transform.rotation.w = np.cos(theta/2) #q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    def pose_estimate_callback(self, msg):
        """
        eghosa's pose callback to display particles but on estimate
        """
        particle_pts = Marker()
        particle_pts.type = Marker.POINTS
        
        particle_pts.header.frame_id = "/map"
        
        particle_pts.scale.x = 0.2
        particle_pts.scale.y = 0.2
        particle_pts.color.a = 1.
        particle_pts.color.r, particle_pts.color.b, particle_pts.color.g = 0.482, 0.431, 0.922
        
        for particle in self.particles:
            p = Point()
            p.x = particle[0]
            p.y = particle[1]
            particle_pts.points.append(p)
        self.particle_estimate_publisher.publish(particle_pts)
        # self.get_logger().info("published particle estimate")

    def pose_callback(self, msg):
        """
        eghosa's pose callback to display initial pose particles (triggered by RVIZ 2d pose estimate)
        """
        std_x = 0.5
        std_y = 0.5
        std_theta = 0.25
        
        x = msg.pose.pose.position.x + 1.0 # try initializing forward a bit
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
        
        particle_pts.scale.x = 0.2
        particle_pts.scale.y = 0.2
        particle_pts.color.a = 1.
        particle_pts.color.r, particle_pts.color.b, particle_pts.color.g = 1.0, 0.0, 0.0
        
        for particle in self.particles:
            p = Point()
            p.x = particle[0]
            p.y = particle[1]
            particle_pts.points.append(p)
        self.particle_publisher.publish(particle_pts)
        print("published initial particles")

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
