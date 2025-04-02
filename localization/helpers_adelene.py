import numpy as np

class PoseTransformTools:
    # @staticmethod
    def __init__(self):
        pass
    def rotation_matrix(self,theta):
            """
            Return a 2D rotation matrix given the angle theta
            Parameters
            ----------
            theta: float
                angle in radians
            Returns
            -------
            R: np.array
                2x2 rotation matrix
            """
            return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def rotation_matrix_to_angle(self,R):
        """
        Return the angle from a 2D rotation matrix
        Parameters
        ----------
        R: np.array
            2x2 rotation matrix
        Returns
        -------
        theta: float
            angle in radians
        """
        theta = np.arctan2(R[1, 0], R[0, 0])
        return theta

    def pose_to_transformation_matrix(self,pose):
        """
        Convert a pose (x, y, theta) to a 3x3 transformation matrix
        
        Parameters
        ----------
        pose: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the pose, and the third element is the angle in radians.
        
        Returns
        -------
        T: np.array
            3x3 transformation matrix
        """
        theta = pose[2]
        t = np.array([[pose[0]], [pose[1]]])  # pose[:2]
        
        R = self.rotation_matrix(theta)
        bottom_row = np.array([0.0, 0.0, 1.0])
        T = np.vstack((np.concatenate((R, t), axis=1), bottom_row))
        # print("T", T)
        return T
    
    
    def poses_to_transformation_matrices(self, poses):
        """
        Converts poses into transformation matrices for all particles at once.
        
        Parameters
        ----------
        poses: np.array
            n x 3 array of poses where each row is [x, y, theta].
        
        Returns
        -------
        np.array
            n x 3x3 array of transformation matrices corresponding to each pose.
        """
        # Extract x, y, theta
        x = poses[:, 0]
        y = poses[:, 1]
        theta = poses[:, 2]

        # Compute transformation matrices using broadcasting
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # Construct the transformation matrices for all particles
        transformation_matrices = np.stack([
            cos_theta, -sin_theta, x,
            sin_theta, cos_theta, y,
            np.zeros_like(x), np.zeros_like(y), np.ones_like(x)
            ], axis=-1).reshape(-1, 3, 3)

        return transformation_matrices
    def transformation_matrix_to_pose(self,T):
        """
        Convert a 3x3 transformation matrix to a pose (x, y, theta)
        
        Parameters
        ----------
        T: np.array
            3x3 transformation matrix
        
        Returns
        -------
        pose: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the pose, and the third element is the angle in radians.
        """
        theta = self.rotation_matrix_to_angle(T[:2, :2])
        t = T[:2, 2]
        return np.concatenate((t, [theta]), axis=0)

    
    def transformation_matrices_to_poses(self, transformation_matrices):
        """
        Converts transformation matrices back to poses for all particles at once.

        Parameters
        ----------
        transformation_matrices: np.array
            n x 3 x 3 array of transformation matrices.

        Returns
        -------
        np.array
            n x 3 array of poses where each row is [x, y, theta].
        """
        # Extract x, y, theta from the transformation matrices
        x = transformation_matrices[:, 0, 2]
        y = transformation_matrices[:, 1, 2]
        
        # Calculate theta using arctan2
        theta = np.arctan2(transformation_matrices[:, 1, 0], transformation_matrices[:, 0, 0])

        # Stack into poses [x, y, theta]
        poses = np.column_stack((x, y, theta))

        return poses

    def get_odometry_1to2(self,pose_1, pose_2):
        """
        Compute the odometry between two poses in the world frame.
        
        Parameters
        ----------
        pose_1: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the first pose, and the third element is the angle in radians.
        pose_2: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the second pose, and the third element is the angle in radians.
        
        Returns
        -------
        deltaX: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the odometry, and the third element is the angle in radians.
        """

        ### X_k-1 -> T_w_r1 "robot r1 in world frame"
        T_w_r1 = self.pose_to_transformation_matrix(pose_1) # transformation matrix from world to robot11
        # print(T_w_r1)

        ### X_k -> T_w_r2 "robot r2 in world frame"
        T_w_r2 = self.pose_to_transformation_matrix(pose_2)
        # print(T_w_r2)

        # Get inverse of T_w_r1
        T_r1_w = np.linalg.inv(T_w_r1)
        # print("inverse", T_r1_w)

        # Multiply to get T_r1_r2
        T_r1_r2 = T_r1_w @ T_w_r2
        
        # Get pose from transformation matrix
        deltaX = self.transformation_matrix_to_pose(T_r1_r2)
        
        return deltaX  # [0, 0, 0]

    def get_new_pose(self,pose_1, deltaX):

        """
        Compute the new pose after moving from pose_1 by the odometry deltaX.
        
        Parameters
        ----------
        pose_1: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the initial pose, and the third element is the angle in radians.
        deltaX: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the odometry, and the third element is the angle in radians.
        
        Returns
        -------
        pose_2: np.array
            3x1 array, where first two elements are the x and y coordinates 
            of the new pose, and the third element is the angle in radians.
        """
        # return pose_1+deltaX
        T_w_r1 = self.pose_to_transformation_matrix(pose_1)
        T_r1_r2 = self.pose_to_transformation_matrix(deltaX)
        
        T_w_r2 = T_w_r1 @ T_r1_r2
        # print("T_w_r2", T_w_r2)
        pose_2 = self.transformation_matrix_to_pose(T_w_r2)  

        return pose_2

    def get_new_poses(self, poses, deltaXs):
        """
        Compute the new poses for multiple particles using their current poses and the odometry deltas.
        
        Parameters
        ----------
        poses: np.array
            n x 3 array of current poses (x, y, theta) for all particles.
        deltaXs: np.array
            n x 3 array of odometry deltas (dx, dy, dtheta) for all particles.

        Returns
        -------
        np.array
            n x 3 array of updated poses (x, y, theta) for all particles.
        """
        # Convert poses and deltas into transformation matrices
        T_w_r1 = self.poses_to_transformation_matrices(poses)  # n x 3 -> n x 3x3 matrices
        T_r1_r2 = self.poses_to_transformation_matrices(deltaXs)  # n x 3 -> n x 3x3 matrices

        # Compute the new transformation matrices
        T_w_r2 = T_w_r1 @ T_r1_r2  # Matrix multiplication to update poses

        # Convert back to poses
        new_poses = self.transformation_matrices_to_poses(T_w_r2)  # Convert matrix back to pose

        return new_poses