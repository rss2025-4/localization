from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from geometry_msgs.msg import Point


from rclpy.node import Node
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
        # initial_particles = np.zeros((100,3)) # for now
        # self.particles = initial_particles

        # from eghosa's code: 
        self.particles = np.zeros((self.NUM_PARTICLES, 3))
        self.particle_publisher = self.create_publisher(Marker, "/particle", 1)
        self.particle_estimate_publisher = self.create_publisher(Marker, "/particle_estimate", 1)
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)
        # self.pose_estimate_sub = self.create_subscription(Odometry, "/pf/pose/odom",
        #                                          self.pose_estimate_callback,
        #                                          1)
        
    def laser_callback(self, msg):
        # pass
        observation = msg.ranges
        self.particle_probabilites = self.sensor_model.evaluate(self.particles, observation)
        
        self.particles = self.resample(self.particle_probabilites, self.particles)
        # get array of most probabilities
        self.get_pose() # publishes pose estimate
        self.get_logger().info("laser callback")
        # pick most 

        # pass
        # self.sensor_model.update_sensor(msg)
    def odom_callback(self, msg):
        # pass
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        current_time = float(msg.header.stamp.nanosec)/1e9
        
        odometry = self.motion_model.update_odometry(x_velocity, y_velocity, angular_velocity, current_time)
        self.particles = self.motion_model.evaluate(self.particles, odometry)
        
        self.get_pose() # publishes pose estimate
        self.get_logger().info("odom callback")
        # pass
    
    def resample(self, particle_probabilities, particles):
        if particle_probabilities is None:
            return self.particles
        # Create list of options to sample from
        options = np(0,len(particle_probabilities)+1,1)
        selections = np.random.choice(particles, len(particles), p=particle_probabilities)
        return self.particles[selections]
        # makes new particles

    def get_pose(self):
        #num particles
        """
        Calculate and publish the average pose of the particles.

        Computes the average x and y positions of the particles, and
        calculates the average orientation using the circular mean 
        for the theta values. Publishes the estimated pose as an 
        Odometry message.

        The published pose is an estimate of the robot's position 
        and orientation in the map frame.

        """

        n = len(self.particles)

        # for x,y, get average  
        x_avg = sum(self.particles[0,:])/len(self.particles)
        y_avg = sum(self.particles[1,:])/len(self.particles)

        # for theta - need to get circular mean
        cos_thetas = np.cos(self.particles[2,:])
        sin_thetas = np.sin(self.particles[2,:])
        mean_cos = sum(cos_thetas)/len(cos_thetas)
        mean_sin = sum(sin_thetas)/len(sin_thetas)
        
        theta_avg = np.arctan2((1/n)*mean_sin,(1/n)*mean_cos) 

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
        # self.get_logger().info("Published transform pf to map")
        # se3_to_tf(mat: np.ndarray, time: Any, parent: str, child: str)

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
    # def se3_to_tf(mat: np.ndarray, time: Any, parent: str, child: str) -> TransformStamped:
    #     """
    #     Convert a 4x4 SE3 matrix to a TransformStamped message
    #     """
    #     obj = TransformStamped()

    #     # current time
    #     obj.header.stamp = time.to_msg()

    #     # frame names
    #     obj.header.frame_id = parent
    #     obj.child_frame_id = child

    #     # translation component
    #     obj.transform.translation.x = mat[0, 3]
    #     obj.transform.translation.y = mat[1, 3]
    #     obj.transform.translation.z = mat[2, 3]

    #     # rotation (quaternion)
    #     q = tf_transformations.quaternion_from_matrix(mat)
    #     obj.transform.rotation.x = q[0]
    #     obj.transform.rotation.y = q[1]
    #     obj.transform.rotation.z = q[2]
    #     obj.transform.rotation.w = q[3]

    #     return obj

    def publish_transform(self,pose):

        # need to switch to map frame instead of laser frame 
        # try:
        #     laser_to_map_msg: TransformStamped = self.tfBuffer.lookup_transform('map', 'laser',
        #                                                                          rclpy.time.Time())
        # except tf2_ros.TransformException:
        #     self.get_logger().info('waiting on parent')
        #     return 

        # laser_to_map_tf = tf_to_se3(laser_to_map_msg.transform)
        # particles_to_map = 
        # particle_to_map 
        [x,y,theta] = pose
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()


        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link_pf'

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
        t.transform.rotation.z = theta #q[2]
        t.transform.rotation.w = 1.0 #q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    def pose_estimate_callback(self, msg):
        """
        eghosa's pose callback to display particles but on estimate
        """
        # std_x = 0.5
        # std_y = 0.5
        # std_theta = 0.25
        
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        # roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
        #                                             msg.pose.pose.orientation.y,
        #                                             msg.pose.pose.orientation.z,
        #                                             msg.pose.pose.orientation.w])
        # xs = np.random.normal(x, std_x, self.NUM_PARTICLES)
        # ys = np.random.normal(y, std_y, self.NUM_PARTICLES)
        # thetas = np.random.normal(yaw, std_theta, self.NUM_PARTICLES)
        # self.particles = np.vstack((xs, ys, thetas)).T
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
        self.particle_estimate_publisher.publish(particle_pts)
        self.get_logger().info("published particle estimate")

    def pose_callback(self, msg):
        """
        eghosa's pose callback to display particles
        """
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
        print("published initial particles")

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
