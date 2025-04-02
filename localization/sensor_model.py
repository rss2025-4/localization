import numpy as np
from scan_simulator_2d import PyScanSimulator2D

# Try to change to just `from scan_simulator_2d import PyScanSimulator2D`
# if any error re: scan_simulator_2d occurs

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid

import sys

np.set_printoptions(threshold=sys.maxsize)


class SensorModel:

    def __init__(self, node):

        self.sensor_node = node
        node.declare_parameter("map_topic", "default")
        node.declare_parameter("num_beams_per_particle", 1)
        node.declare_parameter("scan_theta_discretization", 1.0)
        node.declare_parameter("scan_field_of_view", 1.0)
        node.declare_parameter("lidar_scale_to_map_scale", 1.0)

        self.map_topic = (
            node.get_parameter("map_topic").get_parameter_value().string_value
        )
        self.num_beams_per_particle = (
            node.get_parameter("num_beams_per_particle")
            .get_parameter_value()
            .integer_value
        )
        self.scan_theta_discretization = (
            node.get_parameter("scan_theta_discretization")
            .get_parameter_value()
            .double_value
        )
        self.scan_field_of_view = (
            node.get_parameter("scan_field_of_view").get_parameter_value().double_value
        )
        self.lidar_scale_to_map_scale = (
            node.get_parameter("lidar_scale_to_map_scale")
            .get_parameter_value()
            .double_value
        )

        ####################################
        # Adjust these parameters
        self.alpha_hit = 1  # 0.74
        self.alpha_short = 0  # 0.07
        self.alpha_max = 0  # 0.07
        self.alpha_rand = 0  # 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        node.get_logger().info("%s" % self.map_topic)
        node.get_logger().info("%s" % self.num_beams_per_particle)
        node.get_logger().info("%s" % self.scan_theta_discretization)
        node.get_logger().info("%s" % self.scan_field_of_view)

        # Precompute the sensor model table
        self.sensor_model_table = np.empty((self.table_width, self.table_width))
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization,
        )

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_subscriber = node.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 1
        )

    # def ros_log(node, message):
    #     node.get_logger().info(message)
    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        alpha_hit = self.alpha_hit
        alpha_short = self.alpha_short
        alpha_max = self.alpha_max
        alpha_rand = self.alpha_rand
        sigma = self.sigma_hit

        z_max = self.table_width - 1

        self.sensor_model_table

        # actual distance d and measured distance z_k^i

        ### Normalize p_hit values across columns
        def p_hit_z_k_only(z_k, d):
            """to normalize case 1 only"""
            # Case 1: probability of detecting known obstacle in map
            if z_k <= z_max and z_k >= 0:
                eta = 1
                p_hit_z_k = (
                    eta
                    * 1
                    / (np.sqrt(2 * np.pi * sigma**2))
                    * np.exp(-1 / 2 * (z_k - d) ** 2 / sigma**2)
                )
            else:
                p_hit_z_k = 0

            return p_hit_z_k

        # Iterate through the table and fill up with p_hit values, then normalize
        for z_k in range(self.table_width):  # rows
            for d in range(self.table_width):  # columns
                # Compute the probability of measuring j given d
                self.sensor_model_table[z_k, d] = p_hit_z_k_only(z_k, d)
        self.sensor_model_table /= np.sum(self.sensor_model_table, axis=0)

        ### Calculate the rest of the cases, add back Case 1, and then normalize entire table
        def p_z_k_rest(z_k, d):
            # # Case 1: probability of detecting known obstacle in map
            # if z_k <= z_max and z_k >=0:
            #     eta = 1
            #     p_hit_z_k = eta*1/(np.sqrt(2*np.pi*sigma**2))*np.exp(-1/2*(z_k - d)**2/sigma**2)
            # else:
            #     p_hit_z_k = 0

            # Case 2: probability of detecting short measurement (maybe internal lidar reflections)
            if z_k <= d and z_k >= 0 and d != 0:
                p_short_z_k = 2 / d * (1 - z_k / d)
            else:
                p_short_z_k = 0

            # Case 3: probability of a very large measurement (missed, strange reflection)
            eps = 1  # 1 pixel
            if z_k == z_max:
                p_max_z_k = eps
            else:
                p_max_z_k = 0

            # Case 4: probability of random measurement
            if z_k <= z_max and z_k >= 0:
                p_random_z_k = 1 / (z_max)
            else:
                p_random_z_k = 0

            # Mix the 3 cases

            # alpha_hit*p_hit_z_k +
            p_z_k = (
                alpha_short * p_short_z_k
                + alpha_max * p_max_z_k
                + alpha_rand * p_random_z_k
            )
            return p_z_k

        # Iterate through the table
        for z_k in range(self.table_width):  # rows
            for d in range(self.table_width):  # columns
                # Compute the probability of measuring j given d
                self.sensor_model_table[z_k, d] = (
                    p_z_k_rest(z_k, d) + alpha_hit * self.sensor_model_table[z_k, d]
                )

        # Normalize the table along each column
        self.sensor_model_table /= np.sum(self.sensor_model_table, axis=0)
        # self.sensor_model_table = np.nan_to_num(self.sensor_model_table)
        # raise NotImplementedError

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. THIS IS Z_K. Each range in Z_K is Z_K^i

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle
        node = self.sensor_node
        scans = self.scan_sim.scan(particles) # ray tracing from all the particles
        
        node.get_logger().info("scan %s" % np.size(scans, 0))
        # convert scan and observation from meters to pixel
        # by dividing by map resolution and scale
        scans = np.floor(
            scans / int(self.resolution * self.lidar_scale_to_map_scale)
        ).astype(int)
        observation = np.floor(
            observation / (self.resolution * self.lidar_scale_to_map_scale)
        ).astype(int)

        N = np.size(scans, 0)
        # node.get_logger().info("N %s" % N)
        # node.get_logger().info("converting to pixels scans %s" % np.size(scans, 1))
        # node.get_logger().info(
        #     "converting to pixels observation %s" % np.size(observation, 0)
        # )
        # then clip to z_max and 0
        num_beams = np.size(observation, 0)
        # node.get_logger().info("num_beams %s" % num_beams)

        scans = np.clip(scans, 0, self.table_width - 1)
        observation = np.clip(observation, 0, self.table_width - 1)

        # node.get_logger().info("clipped to pixels scans %s" % np.size(scans, 1))
        # node.get_logger().info(
        #     "clipped to pixels observation %s" % np.size(observation, 0)
        # )

        particle_probabilites = np.zeros((len(particles), 1))
        for j in range(N):
            # scan is a list of the form [d1, d2, d3, ...]
            # where each d_i is the distance from the particle to the closest obstacle
            # in the direction of the i-th beam
            scan = scans[j, :]  # get scan (some j-th row)
            # node.get_logger().info("in for loop, scan %s" % np.size(scan, 0))
            total_p_z_k = 1  # initialize a total probability for each particle
            for i in range(num_beams):  # each measured distance z_k_i
                # use precomputed table to get the probability of each beam
                # given the distance from the map
                # and the distance from the laser scan
                z_k_i = observation[i]
                d_i = scan[i]
                p_z_k_i = self.sensor_model_table[z_k_i, d_i]

                # likelihood of each particle is a product of the likelihoods of each beam
                total_p_z_k = total_p_z_k * p_z_k_i
            # node.get_logger().info("made it through beams")
            particle_probabilites[j, 0] = total_p_z_k
                
        # node.get_logger().info("sum of particle probabilities %s" % np.sum(particle_probabilites))
        # node.get_logger().info("all particle probabilities %s" % particle_probabilites)
        ####################################
        
        
        return particle_probabilites

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.0
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = euler_from_quaternion(
            (origin_o.x, origin_o.y, origin_o.z, origin_o.w)
        )
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5,
        )  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")


# print("Sensor model initialized")
# if __name__ == "__main__":
# sensor_model = SensorModel()
# print(SensorModel.sensor_model_table[0,:])
