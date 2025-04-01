import numpy as np
import math
from .helpers import *

# added for debugging
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped

from ackermann_msgs.msg import AckermannDriveStamped


from rclpy.node import Node
import rclpy

assert rclpy
# [VISUALIZATION] NEW: Added for visualizing the estimated pose marker
def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# [VISUALIZATION] NEW: Placeholder for laser scan transformation function.
def get_laser_scan(estimated_pose):
    # For demonstration: generate scan points in a semicircle in front of the robot.
    angles = np.linspace(-np.pi/4, np.pi/4, 20)
    r = 5.0  # example fixed range
    scan_points = []
    for a in angles:
        x = estimated_pose[0] + r * np.cos(estimated_pose[2] + a)
        y = estimated_pose[1] + r * np.sin(estimated_pose[2] + a)
        scan_points.append([x, y])
    return scan_points

# originally not node  (added for debugging as a ros node "ros2 run localization motion_model")
class MotionModel():  #  for unit tests, remove Node superclass

    def __init__(self,node): # originally (self, node) - for unit tests, make sure "(self, node)" not "(self)"
        ####################################
        # TODO
        # super().__init__("motion_model") # originally not here

        self.debug = False # for visualizing motion model particles

        # Do any precomputation for the motion
        # model here.

        self.deterministic = False # deterministic or non-deterministic, set to True for unit tests!
        
        # need to integrate wheel odometry from integration of motor and steering commands
        self.prev_time = None
        # self.velocity = 0
        # self.steering_angle = 0

        if self.deterministic is False:
            # self.v_std = 0.1
            self.dx_std = 0.1
            self.dy_std = 0.1
            self.theta_std = 0.1
        else:
            # self.v_std = 0.0
            self.dx_std = 0.0
            self.dy_std = 0.0
            self.theta_std = 0.0
        
        

        if self.debug is True:
            self.particles_pub = self.create_publisher(Marker, "/motion_model_particles", 1)
            self.estimated_pose_pub = self.create_publisher(Marker, "/estimated_pose", 1) 
            self.laser_pub = self.create_publisher(Marker, "/transformed_laser_scan", 1) 
            self.odom_sub = self.create_subscription(Odometry, "/odom",
                                                 self.odom_callback,
                                                 1)
        
            # self.initial_particles = np.zeros((100,3)) # just initalizing 100 particles at origin (0,0) with heading of 0
            # self.particles = self.initial_particles
            self.particles = np.zeros((100,3)) # initial particles at origin [0,0,0]
            
            self.logged_particles = []
            self.logged_estimated_pose = []
            self.logged_odom = []
            self.logged_laser = []

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
        print("x_velocity, y_velocity, angular_velocity", x_velocity, y_velocity, angular_velocity)
        # Calculate the change in pose given the instantaneous velocities
        if self.prev_time is None:
            # first instance, so no time difference yet
            self.prev_time = current_time
            print("initial")
            return [0.0, 0.0, 0.0]

        dt = current_time - self.prev_time # should be in seconds
        dx = x_velocity * dt
        dy = y_velocity * dt
        dtheta = angular_velocity * dt

        print("dt", dt)

        self.prev_time = current_time
        return [dx, dy, dtheta]
    
    def odometry_from_ackermann_motion_model_vel(self, pose1, velocity, theta, particles):
        """
        NOT USED 
        Old odometry calculation - using velocity (2 params, v and theta)
        """
        x_prev,y_prev,theta_prev = pose1

        # calculate gaussian distribution about velocity and theta
        # syntax: np.random.normal(mean, std)
        v_distribution = np.random.normal(velocity, self.v_std, len(particles))
        theta_distribution = np.random.normal(theta, self.theta_std, len(particles))
        
        # calculate a new pose based on sample from distribution (??)
        v_sample = np.random.choice(v_distribution)
        theta_sample = np.random.choice(theta_distribution)

        # calculate new pose with noise-injected odometry
            # notice theta independent of x,y but not vice versa
        x = x_prev + v_sample * np.cos(theta_sample)
        y = y_prev + v_sample * np.sin(theta_sample)  
        theta = theta_prev + theta_sample
        
        pose2 = [x,y,theta]
        # possible_x = x_prev + 

        return pose2

    def odometry_from_ackermann_motion_model_pose(self, pose1, odometry, particles):
        "Calculates new possible poses from odometry with noise (3 params, dx, dy, dtheta)"

        x_prev,y_prev,theta_prev = pose1
        dx, dy, dtheta = odometry[0], odometry[1], odometry[2]
    
        # Calculate gaussian distribution about velocity and theta
        # syntax: np.random.normal(mean, std)
        # v_x = velocity * np.cos(theta)
        # v_y = velocity * np.sin(theta)
        dx_distribution = np.random.normal(dx, self.dx_std, len(particles))
        dy_distribution = np.random.normal(dy, self.dy_std, len(particles))
        theta_distribution = np.random.normal(dtheta, self.theta_std, len(particles))
        
        # Calculate a new deltaX based on chosen sample from noise distribution
        dx_sample = np.random.choice(dx_distribution)
        dy_sample = np.random.choice(dy_distribution)
        theta_sample = np.random.choice(theta_distribution)

        deltaX = [dx_sample, dy_sample, theta_sample]
        # print("deltaX", deltaX)
        # Calculate new pose with noise-injected odometry
        pose2 = get_new_pose(pose1, deltaX) # outputs [x,y,theta]

        return pose2


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
        # updated_particles = np.zeros((len(particles), 3)) # avoid initializing new arrays for memory-sake
        # print("size", np.size(updated_particles))

        for i in range(len(particles)): # for each particles

            # use a function call to update each particle (? room for efficiency improvement?)
            particles[i] = self.odometry_from_ackermann_motion_model_pose(particles[i], odometry, particles)
                    
        return particles # should probably modify array instead of making new one (okay?)

    def odom_callback(self,msg):
        # FOR DEBUGGING
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        current_time = float(msg.header.stamp.nanosec)/1e9
        print("current time", current_time)
        
        [dx, dy, dtheta] = self.update_odometry(x_velocity, y_velocity, angular_velocity, current_time)
        print("dx, dy, dtheta", dx, dy, dtheta)

        all_points = Marker()
        all_points.type = Marker.POINTS
        all_points.header.frame_id = "/laser"
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
        # [VISUALIZATION] NEW: Log data for offline analysis
        self.logged_particles.append(np.copy(self.particles))
        estimated_pose = np.mean(self.particles, axis=0)
        self.logged_estimated_pose.append(estimated_pose)
        self.logged_odom.append([dx, dy, dtheta])

        # [VISUALIZATION] NEW: Publish Particle Markers for RViz
        particle_marker = Marker()
        particle_marker.header.frame_id = "map"  # set appropriate frame
        particle_marker.header.stamp = msg.header.stamp
        particle_marker.ns = "particles"
        particle_marker.id = 0
        particle_marker.type = Marker.POINTS
        particle_marker.action = Marker.ADD
        particle_marker.scale.x = 0.1
        particle_marker.scale.y = 0.1
        particle_marker.color.a = 1.0
        particle_marker.color.r = 0.0
        particle_marker.color.g = 0.0
        particle_marker.color.b = 1.0
        for particle in self.particles:
            p = Point()
            p.x = particle[0]
            p.y = particle[1]
            p.z = 0.0
            particle_marker.points.append(p)
        self.particles_pub.publish(particle_marker)

        # [VISUALIZATION] NEW: Publish Estimated Pose Marker as an Arrow
        estimated_marker = Marker()
        estimated_marker.header.frame_id = "map"
        estimated_marker.header.stamp = msg.header.stamp
        estimated_marker.ns = "estimated_pose"
        estimated_marker.id = 0
        estimated_marker.type = Marker.ARROW
        estimated_marker.action = Marker.ADD
        estimated_marker.pose.position.x = estimated_pose[0]
        estimated_marker.pose.position.y = estimated_pose[1]
        estimated_marker.pose.position.z = 0.0
        quat = euler_to_quaternion(0, 0, estimated_pose[2])
        estimated_marker.pose.orientation.x = quat[0]
        estimated_marker.pose.orientation.y = quat[1]
        estimated_marker.pose.orientation.z = quat[2]
        estimated_marker.pose.orientation.w = quat[3]
        estimated_marker.scale.x = 0.5  # arrow length
        estimated_marker.scale.y = 0.1
        estimated_marker.scale.z = 0.1
        estimated_marker.color.a = 1.0
        estimated_marker.color.r = 1.0
        estimated_marker.color.g = 0.0
        estimated_marker.color.b = 0.0
        self.estimated_pose_pub.publish(estimated_marker)

        # [VISUALIZATION] NEW: Transform and publish Laser Scan Marker
        laser_scan_points = get_laser_scan(estimated_pose)  # Replace with actual transformation if available
        laser_marker = Marker()
        laser_marker.header.frame_id = "map"
        laser_marker.header.stamp = msg.header.stamp
        laser_marker.ns = "laser_scan"
        laser_marker.id = 0
        laser_marker.type = Marker.POINTS
        laser_marker.action = Marker.ADD
        laser_marker.scale.x = 0.05
        laser_marker.scale.y = 0.05
        laser_marker.color.a = 1.0
        laser_marker.color.r = 0.0
        laser_marker.color.g = 1.0
        laser_marker.color.b = 1.0
        for pt in laser_scan_points:
            point = Point()
            point.x = pt[0]
            point.y = pt[1]
            point.z = 0.0
            laser_marker.points.append(point)
        self.laser_pub.publish(laser_marker)
        self.logged_laser.append(laser_scan_points)

    def drive_callback(self):
        # FOR DEBUGGING 
        ####################################
        # declare msg
        drive_msg = AckermannDriveStamped()

        curr_time = self.get_clock().now()

        # assign values
        drive_msg.header.stamp = curr_time.to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = 0.0  # rad
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