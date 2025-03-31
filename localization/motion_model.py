import numpy as np

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

# originally not node  
class MotionModel(Node):

    def __init__(self,node): #originally (self, node)
        ####################################
        # TODO
        super().__init__("motion_model") # originally not here

        # Do any precomputation for the motion
        # model here.
        self.deterministic = True # deterministic or non-deterministic
        # need to integrate wheel odometry from integration of motor and steering commands
        self.prev_time = 0
        self.velocity = 0
        self.steering_angle = 0

        if self.deterministic is False:
            # self.v_std = 0.1
            self.dx_std = 0.1
            self.dy_std = 0.2
            self.theta_std = 0.4
        else:
            self.v_std = 0.0
            self.dx_std = 0.0
            self.dy_std = 0.0
            self.theta_std = 0.0
        
        self.particles_pub = self.create_publisher(Marker, "/motion_model_particles", 1)
        # pass

        self.odom_sub = self.create_subscription(Odometry, "/odom",
                                                 self.odom_callback,
                                                 1)
        
        initial_particles = np.zeros((100,3))
        self.particles = initial_particles

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
        
        also get the previous pose?

        odometry is in reference frame of k-1 body frame

        use ackermann model

        x_dot = v*cos(theta)
        y_dot = v*sin(theta)
        theta_dot = v/L*tan(delta) where delta is steering angle

        pg 110 of PR book

        need deterministic result and non-determinstic result (with noise)
        """

        # 1. Calculate the change in pose given the instantaneous velocities
        # dtheta = np.arctan2()
        dt = current_time - self.prev_time #should already be in seconds
        dx = x_velocity * dt
        dy = y_velocity * dt
        dtheta = angular_velocity * dt

        # also get velocity and steering angle  
        self.velocity = np.sqrt(x_velocity**2 + y_velocity**2) # also equal to x_velocity/cos(dtheta)?
        self.steering_angle = dtheta
        # 2. Use prev pose and current pose to calculate probabilities
        # x_prev = 0
        # y_prev = 0
        # theta_prev = 0

        # using robot state to get odometry form /odom topic
        # use ackermann model
        

        self.prev_time = current_time
        return [dx, dy, dtheta]
    
    # def odometry_from_odom_motion_model(pose1, pose2):
    #     # calc odometry info (odometry model, not velocity) from pose1 to pose2
    #     x,y,theta = pose1
    #     xp,yp,thetap = pose2

    #     dtrans = np.sqrt((xp-x)**2 + (yp-y)**2)
    #     drot1 = np.arctan2(yp-y, xp-x) - theta
    #     drot2 = thetap - theta - drot1
    #     return [dtrans, drot1, drot2]

    # def noise_from_odom_motion_model(pose1, pose2):
    #     dtrans, drot1, drot2 = odometry_from_odom_motion_model(pose1, pose2)

    def odometry_from_ackermann_motion_model(self, pose1, velocity, theta, particles):
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
        """ 3 params: x,y, theta"""
        x_prev,y_prev,theta_prev = pose1
        dx, dy, dtheta = odometry[0], odometry[1], odometry[2]
    
        # calculate gaussian distribution about velocity and theta
        # syntax: np.random.normal(mean, std)
        # v_x = velocity * np.cos(theta)
        # v_y = velocity * np.sin(theta)

        dx_distribution = np.random.normal(dx, self.dx_std, len(particles))
        dy_distribution = np.random.normal(dy, self.dy_std, len(particles))
        theta_distribution = np.random.normal(dtheta, self.theta_std, len(particles))
        
        # calculate a new pose based on sample from distribution (??)
        dx_sample = np.random.choice(dx_distribution)
        dy_sample = np.random.choice(dy_distribution)
        theta_sample = np.random.choice(theta_distribution)

        # calculate new pose with noise-injected odometry
            # notice theta independent of x,y but not vice versa
        # x = x_prev + dx_sample
        # y = y_prev + dy_sample
        # theta = theta_prev + theta_sample

        deltaX = [dx_sample, dy_sample, theta_sample]
        pose2 = get_new_pose(pose1, deltaX) #[x,y,theta]
        # possible_x = x_prev + 

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
        
        ///////
        x_dot = v*cos(theta)
        y_dot = v*sin(theta)
        theta_dot = v/L*tan(delta) where delta is steering angle

        """

        ####################################
        # TODO
        
        updated_particles = np.zeros((len(particles), 3))
        # print("size", np.size(updated_particles))

        for i in range(len(particles)): # for each particles

            # if only deterministic
            # if self.deterministic:
            #     updated_particles[i] = get_new_pose(particles[i],odometry)
           
            # else: # if non-deterministic
                # add noise to the odometry
                # try odometry motion model
                # updated_particles[i] = self.odometry_from_ackermann_motion_model_pose(particles[i], self.velocity, self.steering_angle, particles)
            updated_particles[i] = self.odometry_from_ackermann_motion_model_pose(particles[i], odometry, particles)
                # pass

        return updated_particles
        # raise NotImplementedError
    def odom_callback(self,msg):
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        angular_velocity = msg.twist.twist.angular.z
        current_time = msg.header.stamp.sec
        
        [dx, dy, dtheta] = self.update_odometry(x_velocity, y_velocity, angular_velocity, current_time)

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
        updated_particles = self.evaluate(self.particles,[dx, dy, dtheta])

        print("updated particle 1", updated_particles[0])
        for particle in updated_particles:
            p = Point()
            p.x = particle[0] #[x,y,theta]
            p.y = particle[1]
            # p.z = 0
            all_points.points.append(p)

        self.particles_pub.publish(all_points)
        self.get_logger().info("published particles")
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