#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations

class ErrorPlotter(Node):
    def __init__(self):
        super().__init__('error_plotter')
        
        # Subscribe to the estimated pose and ground truth pose topics.
        self.estimated_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/estimated_pose',
            self.estimated_pose_callback,
            10)
        
        # For ground truth, assuming a transform is available. 
        # Alternatively, subscribe directly to a ground truth topic if available.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.estimated_pose = None
        self.ground_truth_pose = None
        
        self.errors = []
        self.timestamps = []
        
        # Timer to periodically check for transforms and compute error.
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def estimated_pose_callback(self, msg):
        # Convert PoseWithCovarianceStamped to [x, y, theta]
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.estimated_pose = np.array([pos.x, pos.y, yaw])
        
    def timer_callback(self):
        # Try to get the ground truth transform between /map and /base_link.
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pos = trans.transform.translation
            # Assuming ground truth orientation is not used here.
            self.ground_truth_pose = np.array([pos.x, pos.y])
        except Exception as e:
            self.get_logger().info(f'Could not get ground truth transform: {e}')
            return
        
        # Compute error if both poses are available.
        if self.estimated_pose is not None and self.ground_truth_pose is not None:
            est_xy = self.estimated_pose[:2]
            error = np.linalg.norm(est_xy - self.ground_truth_pose)
            self.errors.append(error)
            self.timestamps.append(self.get_clock().now().nanoseconds * 1e-9)
            self.get_logger().info(f'Current error: {error:.3f}')
            
    def plot_error(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.timestamps, self.errors, label='Pose Error')
        plt.xlabel('Time (s)')
        plt.ylabel('Euclidean Error (m)')
        plt.title('Pose Estimation Error vs. Ground Truth')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    error_plotter = ErrorPlotter()
    
    try:
        rclpy.spin(error_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        # When shutting down, plot the collected error data.
        error_plotter.plot_error()
        error_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
