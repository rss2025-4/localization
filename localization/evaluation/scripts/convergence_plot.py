#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

class ConvergencePlotter(Node):
    def __init__(self):
        super().__init__('convergence_plotter')
        self.initial_time = None  # This will hold the time (in seconds) when the initial /particle is received.
        self.times = []           # Relative times (in seconds) of each /particle_estimate message.
        self.std_xs = []          # Standard deviation of x values for each /particle_estimate message.
        self.std_ys = []          # Standard deviation of y values for each /particle_estimate message.
        self.std_thetas = []
        self.initial_received = False

        # Subscribe to the initial particle topic (/particle)
        self.create_subscription(Marker, '/particle', self.particle_callback, 10)
        # Subscribe to the particle estimate topic (/particle_estimate)
        self.create_subscription(Marker, '/particle_estimate', self.particle_estimate_callback, 10)
        self.get_logger().info('ConvergencePlotter node started.')

    def particle_callback(self, msg):
        # Only process the first /particle message (initial pose)
        if not self.initial_received:
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            now = sec + nsec * 1e-9 
            self.initial_time = now
            self.initial_received = True
            xs = [p.x for p in msg.points]
            ys = [p.y for p in msg.points]
            thetas = [p.z for p in msg.points]
            std_x = np.std(xs)
            std_y = np.std(ys)
            std_theta = np.std(thetas)
            self.get_logger().info(f'Initial /particle message received: std_x = {std_x:.3f}, std_y = {std_y:.3f}')
        else:
            self.get_logger().info('Ignoring additional /particle messages.')

    def particle_estimate_callback(self, msg):
        if not self.initial_received:
            # Ignore estimates until the initial pose has been set
            # self.get_logger().info('Ignoring /particle_estimate message before initial /particle.')
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            now = sec + nsec * 1e-9
            self.get_logger().info('Initial particle message recieved at {now} seconds.')
            self.initial_time = now
            self.initial_received = True
            xs = [p.x for p in msg.points]
            ys = [p.y for p in msg.points]
            thetas = [p.z for p in msg.points]
            std_x = np.std(xs)
            std_y = np.std(ys)
            std_theta = np.std(thetas)
            
            self.std_xs.append(std_x)
            self.std_ys.append(std_y)
            self.std_thetas.append(std_theta)
            self.times.append(0)
            self.get_logger().info(f'Initial /particle message received: std_x = {std_x:.3f}, std_y = {std_y:.3f}')
            return
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        now = sec + nsec * 1e-9
        rel_time = now - self.initial_time
        xs = [p.x for p in msg.points]
        ys = [p.y for p in msg.points]
        std_x = np.std(xs)
        std_y = np.std(ys)
        std_theta = np.std([p.z for p in msg.points])
        self.times.append(rel_time)
        self.std_xs.append(std_x)
        self.std_ys.append(std_y)
        self.std_thetas.append(std_theta)
        self.get_logger().info(f'Time: {rel_time}s | std_x: {std_x:.3f} | std_y: {std_y:.3f}')
        # HACK to stop the node after 10 seconds
        if rel_time > 10:
            self.get_logger().info('Stopping data collection after 10 seconds.')
            raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    plotter = ConvergencePlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info("KeyboardInterrupt detected, shutting down node...")
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

        # Plot the convergence data after shutdown
        try:
            if plotter.times:
                fig, ax = plt.subplots()
                ax.plot(plotter.times, plotter.std_xs, label='Standard Deviation of X')
                ax.plot(plotter.times, plotter.std_ys, label='Standard Deviation of Y')
                ax.plot(plotter.times, plotter.std_thetas, label='Standard Deviation of Theta')
                ax.set_xlabel('Time (s) since initial /particle')
                ax.set_ylabel('Standard Deviation')
                ax.set_title('Convergence Plot of Particle Filter')
                ax.legend()
                ax.grid(True)
                
                ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: f'{int(x)}'))

                plt.savefig("convergence_plot_sim_noisy_0.01_0.01_10.png")
            else:
                print("No /particle_estimate messages were received to plot.")
        except Exception as e:
            print(f"Error during plotting: {e}")
            print(f"these are the std xs {plotter.std_xs} and std ys {plotter.std_ys} and std thetas {plotter.std_thetas}")

if __name__ == '__main__':
    main()
