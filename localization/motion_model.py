import numpy as np

def normalize_angle(angle):
    """
    Normalize an angle to be within the range [-pi, pi].
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class MotionModel:

    def __init__(self, node):
        ####################################
        # Do any precomputation for the motion
        # model here.

        self.alpha1 = 0.1  
        self.alpha2 = 0.1          
        self.alpha3 = 0.1  
        self.alpha4 = 0.1  

        ####################################

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

        dx, dy, dtheta = odometry
        num_particles = particles.shape[0]
        new_particles = np.copy(particles)
        for i in range(num_particles):
            x, y, theta = particles[i]
            global_dx = dx * np.cos(theta) - dy * np.sin(theta)
            global_dy = dx * np.sin(theta) + dy * np.cos(theta)
            trans_noise_std = self.alpha2 * np.hypot(dx, dy) + self.alpha1 * abs(dtheta)
            rot_noise_std = self.alpha4 * np.hypot(dx, dy) + self.alpha3 * abs(dtheta)
            noisy_dx = global_dx + np.random.normal(0, trans_noise_std)
            noisy_dy = global_dy + np.random.normal(0, trans_noise_std)
            noisy_dtheta = dtheta + np.random.normal(0, rot_noise_std)
            new_x = x + noisy_dx
            new_y = y + noisy_dy
            new_theta = normalize_angle(theta + noisy_dtheta)
            new_particles[i, 0] = new_x
            new_particles[i, 1] = new_y
            new_particles[i, 2] = new_theta

        ####################################
        return new_particles
        ####################################
