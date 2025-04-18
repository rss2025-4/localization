import numpy as np

from .helpers import PoseTransformTools

# added for debugging
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from ackermann_msgs.msg import AckermannDriveStamped

import time
from rclpy.node import Node
import rclpy

assert rclpy

# originally not node  (added for debugging as a ros node "ros2 run localization motion_model")
class MotionModel():  #  for unit tests, remove Node superclass

    def __init__(self,node): # originally (self, node) - for unit tests, make sure "(self, node)" not "(self)"
        ####################################
        # TODO
        self.node = node
        # super().__init__("motion_model") # originally not here
        self.pose_transform_tools = PoseTransformTools()
        self.debug = False # for visualizing motion model particles IN SIM

        # Do any precomputation for the motion
        # model here.

        self.deterministic = False # deterministic or non-deterministic, set to True for unit tests!
        
        # need to integrate wheel odometry from integration of motor and steering commands
        self.prev_time = None
        # self.velocity = 0
        # self.steering_angle = 0
        self.temp = None

        if self.deterministic is False:
            # self.v_std = 0.1
            self.dx_std = 0.01
            self.dy_std = 0.01
            self.theta_std = 0.01
        else:
            # self.v_std = 0.0
            self.dx_std = 0.0
            self.dy_std = 0.0
            self.theta_std = 0.0
        
        

        if self.debug is True:
            self.particles_pub = self.create_publisher(Marker, "/motion_model_particles", 1)

            self.odom_sub = self.create_subscription(Odometry, "/odom",
                                                 self.odom_callback,
                                                 1)
            self.NUM_PARTICLES = 200
            
            self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.initial_pose_callback,
                                                 1)

            self.particles = np.zeros((self.NUM_PARTICLES,3)) # initial particles at origin [0,0,0]

            self.drive_publisher = self.create_publisher(
                AckermannDriveStamped, "/drive", 10
            )

            drive_timer_period = 1 / 60  # seconds, 60 Hz
            self.drive_timer = self.create_timer(drive_timer_period, self.drive_callback)
        ####################################

    ## helper functions

    def update_odometry(self, x_velocity, y_velocity, angular_velocity, current_time):
        """ Gets velocity odometry from /odom topic - referenced in particle_filter.py 
        and outputs pose transformation odometry from k to k-1 
        
        odometry is in reference frame of k-1 body frame

        use ackermann model (pg 110 of PR book has other models)
        """
        # print("x_velocity, y_velocity, angular_velocity", x_velocity, y_velocity, angular_velocity)
        # Calculate the change in pose given the instantaneous velocities
        # current_time = time.perf_counter()
        # print("current time", current_time)
        # current_time_msg = self.get_clock().now()
        # seconds, nanoseconds = current_time_msg.seconds_nanoseconds()
        # current_time = seconds + nanoseconds * 1e-9

        if self.prev_time is None:
            # first instance, so no time difference yet
            self.prev_time = current_time
            # print("initial")
            return [0.0, 0.0, 0.0]
       
        dt = current_time - self.prev_time # should be in seconds
        if dt < 0:
            # raise ValueError 
            exit() 
            
        if self.node.is_sim:
            dx = x_velocity * dt
            dy = y_velocity * dt
            dtheta = angular_velocity * dt
        else:
            dx = -x_velocity * dt
            dy = -y_velocity * dt
            dtheta = -angular_velocity * dt
        

        # print("dt", dt)

        self.prev_time = current_time

        
        return [dx, dy, dtheta]

    def odometry_from_ackermann_motion_model_pose(self, particles, odometry):
        "Calculates new possible poses from odometry with noise (3 params, dx, dy, dtheta)"

        dx, dy, dtheta = odometry[0], odometry[1], odometry[2]
    
        # Calculate gaussian distribution about velocity and theta
        # syntax: np.random.normal(mean, std)
        dx_distribution = np.random.normal(dx, self.dx_std, len(particles))
        dy_distribution = np.random.normal(dy, self.dy_std, len(particles))
        theta_distribution = np.random.normal(dtheta, self.theta_std, len(particles))
        
        deltaX_all = np.stack((dx_distribution, dy_distribution, theta_distribution), axis=-1)

        x_prev, y_prev, theta_prev = np.array(particles).T
        
        new_poses = self.pose_transform_tools.get_new_poses(np.array([x_prev, y_prev, theta_prev]).T, deltaX_all)
        return new_poses


    def initial_pose_callback(self, msg):
        """
        Using Eghosa's pose callback to display initial particles (triggered by RVIZ 2D pose estimate)
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


    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        # updates all particles in one call
        updated_particles = self.odometry_from_ackermann_motion_model_pose(particles, odometry)
        
        return updated_particles


    def odom_callback(self,msg):
        # FOR DEBUGGING
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        current_time_msg = self.get_clock().now()
        seconds, nanoseconds = current_time_msg.seconds_nanoseconds()
        current_time = seconds + nanoseconds * 1e-9


        if self.temp is None: 
            self.temp = current_time
        print("previous time", self.prev_time)
        print("current time", current_time)
        
        if current_time < self.temp:
            return
            # exit()
        self.temp = current_time

        [dx, dy, dtheta] = self.update_odometry(x_velocity, y_velocity, angular_velocity, current_time)
        print("dx, dy, dtheta", dx, dy, dtheta)

         
        all_points = Marker()
        all_points.type = Marker.POINTS
        all_points.header.frame_id = "/map"
        color = (1., 0., 0.)
        all_points.scale.x = 0.1
        all_points.scale.y = 0.1
        all_points.color.a = 1.
        all_points.color.r = color[0]
        all_points.color.g = color[1]
        all_points.color.g = color[2]
        
        # updated_particles = [[0.0,1.0,0.0],[0.0,2.0,0.0]]#self.motion_model.evaluate()
        self.particles = self.evaluate(self.particles,[dx, dy, dtheta])

        print("updated particle 1", self.particles[0])
        for particle in self.particles:
            p = Point()
            p.x = particle[0] #[x,y,theta]
            p.y = particle[1]
            # p.z = 0
            all_points.points.append(p)

        self.particles_pub.publish(all_points)
        self.get_logger().info("published particles")
        print("first particle", self.particles[0])
    def drive_callback(self):
        # FOR DEBUGGING 
        ####################################
        # declare msg
        drive_msg = AckermannDriveStamped()

        curr_time = self.get_clock().now()

        # assign values
        drive_msg.header.stamp = curr_time.to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = 1.0  # rad
        drive_msg.drive.steering_angle_velocity = 0.0  # rad/s
        drive_msg.drive.speed = 0.5  # self.VELOCITY  # 2.0  # m/s
        drive_msg.drive.acceleration = 0.0  # m/s^2
        drive_msg.drive.jerk = 0.0  # m/s^3

        # publish message
        # if not self.stop:
        self.drive_publisher.publish(drive_msg)
        ####################################

def main(args=None):
    rclpy.init(args=args)
    mm = MotionModel()
    rclpy.spin(mm)
    rclpy.shutdown()