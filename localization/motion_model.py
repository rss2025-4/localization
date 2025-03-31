import numpy as np

class MotionModel:

    def __init__(self, node):
        """
        Initialize the motion model parameters
        
        args:
            node: The ROS node object
        """
        # Config parameters for motion model noise
        self.alpha1 = 0.1  # Noise in rotation from rotation
        self.alpha2 = 0.1  # Noise in rotation from translation
        self.alpha3 = 0.1  # Noise in translation from translation
        self.alpha4 = 0.1  # Noise in translation from rotation
        
        # Flag to control whether the model is deterministic (for testing)
        self.deterministic = False
        
        # Store a reference to the node for parameter access/logging
        self.node = node

        

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

        ####################################
        # Make a copy of particles to avoid modifying the original
        updated_particles = np.copy(particles)
        
        # Extract the odometry components
        dx, dy, dtheta = odometry
        
        # Number of particles
        N = particles.shape[0]
        
        # Convert the raw odometry to robot-centric motion
        # Calculate the translation and rotation from odometry
        trans = np.sqrt(dx**2 + dy**2)
        rot1 = np.arctan2(dy, dx) if trans > 1e-5 else 0.0
        rot2 = dtheta - rot1
        
        # Apply the motion model to each particle
        for i in range(N):
            x, y, theta = particles[i]
            
            # Add noise to the motion parameters if not deterministic
            if not self.deterministic:
                # Sample noise based on motion model parameters
                sigma_rot1 = self.alpha1 * abs(rot1) + self.alpha2 * trans
                sigma_trans = self.alpha3 * trans + self.alpha4 * (abs(rot1) + abs(rot2))
                sigma_rot2 = self.alpha1 * abs(rot2) + self.alpha2 * trans
                
                # Add noise
                rot1_noisy = rot1 + np.random.normal(0, sigma_rot1)
                trans_noisy = trans + np.random.normal(0, sigma_trans)
                rot2_noisy = rot2 + np.random.normal(0, sigma_rot2)
            else:
                # For deterministic testing, use exact values
                rot1_noisy = rot1
                trans_noisy = trans
                rot2_noisy = rot2
            
            # Apply the motion model to this particle
            theta_new = theta + rot1_noisy
            x_new = x + trans_noisy * np.cos(theta_new)
            y_new = y + trans_noisy * np.sin(theta_new)
            theta_new = theta_new + rot2_noisy
            
            # Normalize the orientation to [-π, π]
            theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
            
            # Update this particle
            updated_particles[i] = [x_new, y_new, theta_new]
        
        return updated_particles

        ####################################
