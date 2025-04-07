import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import Buffer, TransformListener
import matplotlib.pyplot as plt
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class NoisyOdomNode(Node):
    def __init__(self):
        super().__init__('noisy_odom')
        
        # Parameters for noise levels
        self.declare_parameter('linear_velocity_noise_std', 0.1)  # m/s
        self.declare_parameter('angular_velocity_noise_std', 0.1)  # rad/s
        
        # Get noise parameters
        self.linear_noise_std = self.get_parameter('linear_velocity_noise_std').value
        self.angular_noise_std = self.get_parameter('angular_velocity_noise_std').value
        
        # Subscribe to original odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
            
        # Publisher for noisy odometry
        self.noisy_odom_pub = self.create_publisher(
            Odometry, 
            '/noisy_odom',
            10)
        
        # For ground truth comparison
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # For error plotting
        self.pose_errors = []
        self.timestamps = []
        self.started = False
        # self.start_time = self.get_clock().now()
        self.create_timer(0.1, self.check_duration)

        self.noisy_x = 0.0
        self.noisy_y = 0.0
        self.noisy_theta = 0.0
        self.last_time = None
        self.get_logger().info(f'Noisy Plotter node started, linear noise is {self.linear_noise_std}, angular noise is {self.angular_noise_std}')

    def check_duration(self):
        if self.started:
            if (self.get_current_time() - self.start_time) > 10.0:
                # Plot final results
                if len(self.pose_errors) > 0:
                    self.plot_errors()
                    self.get_logger().info('Saved final plot after 10 seconds')
                rclpy.shutdown()

    def get_current_time(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        now = sec + nsec * 1e-9
        return now

    def odom_callback(self, msg):
        # Add noise to velocities
        noisy_msg = Odometry()
        noisy_msg.header = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id
        
        # Add Gaussian noise to velocities
        noisy_linear_x = msg.twist.twist.linear.x + np.random.normal(0, self.linear_noise_std)
        noisy_linear_y = msg.twist.twist.linear.y + np.random.normal(0, self.linear_noise_std)
        noisy_angular_z = msg.twist.twist.angular.z + np.random.normal(0, self.angular_noise_std)
        
        # Set noisy velocities
        noisy_msg.twist.twist.linear.x = noisy_linear_x
        noisy_msg.twist.twist.linear.y = noisy_linear_y
        noisy_msg.twist.twist.angular.z = noisy_angular_z
        
        # Integrate velocities to get noisy pose
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is not None:
            dt = current_time - self.last_time
            
            # Update pose using noisy velocities
            self.noisy_theta += noisy_angular_z * dt
            self.noisy_x += (noisy_linear_x * np.cos(self.noisy_theta) - 
                           noisy_linear_y * np.sin(self.noisy_theta)) * dt
            self.noisy_y += (noisy_linear_x * np.sin(self.noisy_theta) + 
                           noisy_linear_y * np.cos(self.noisy_theta)) * dt
        
        self.last_time = current_time
        
        # Set integrated noisy pose
        noisy_msg.pose.pose.position.x = self.noisy_x
        noisy_msg.pose.pose.position.y = self.noisy_y
        
        # Publish noisy odometry
        self.noisy_odom_pub.publish(noisy_msg)
        
        # # Compare to ground truth
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
                
            # Calculate pose error using noisy pose
            error = np.sqrt(
                (transform.transform.translation.x - self.noisy_x)**2 +
                (transform.transform.translation.y - self.noisy_y)**2
            )
            
            self.pose_errors.append(error)
            
            self.get_logger().info(f'Successfully published noisy odometry with error: {error:.3f}')
            if not self.started:
                self.get_logger().info(f'Noisy odometry started at {current_time:.3f} seconds')
                self.started = True
                
                self.start_time = current_time
            self.timestamps.append(current_time)
                
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {str(e)}')
    # def pose_estimate_callback(self, msg):
    #     # Compare to ground truth
    #     try:
    #         transform = self.tf_buffer.lookup_transform(
    #             'map',
    #             'base_link',
    #             rclpy.time.Time())
                
    #         # Calculate pose error using noisy pose
    #         error = np.sqrt(
    #             (transform.transform.translation.x - self.noisy_x)**2 +
    #             (transform.transform.translation.y - self.noisy_y)**2
    #         )
            
    #         self.pose_errors.append(error)
            
    #         self.get_logger().info(f'Successfully published noisy odometry with error: {error:.3f}')
    #         if not self.started:
    #             self.get_logger().info(f'Noisy odometry started at {current_time:.3f} seconds')
    #             self.started = True
    #             self.start_time = current_time
    #         self.timestamps.append(current_time)
                
    #     except Exception as e:
    #         self.get_logger().warn(f'Could not get transform: {str(e)}')
    def plot_errors(self):
        print("Plotting errors...")
        print(len(self.pose_errors), len(self.timestamps))
        
        plt.figure(figsize=(10, 6))
        plt.plot(self.timestamps, self.pose_errors)
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (m)')
        plt.title(f'Position Error vs Time\nNoise std: linear={self.linear_noise_std:.2f}, angular={self.angular_noise_std:.2f}')
        plt.grid(True)
        plt.savefig(f'pose_error_l{self.linear_noise_std}_a{self.angular_noise_std}.png')
        self.get_logger().info('Pose error plot saved.')
        plt.show()
        plt.savefig(f'pose_error_l{self.linear_noise_std}_a{self.angular_noise_std}.png')
        plt.close()

def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()