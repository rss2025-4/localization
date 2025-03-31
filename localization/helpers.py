import numpy as np

# class TransformTools:
# @staticmethod
def rotation_matrix(theta):
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

def rotation_matrix_to_angle(R):
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

def pose_to_transformation_matrix(pose):
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
    
    R = rotation_matrix(theta)
    bottom_row = np.array([0.0, 0.0, 1.0])
    T = np.vstack((np.concatenate((R, t), axis=1), bottom_row))
    # print("T", T)
    return T
def transformation_matrix_to_pose(T):
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
    theta = rotation_matrix_to_angle(T[:2, :2])
    t = T[:2, 2]
    return np.concatenate((t, [theta]), axis=0)
def get_odometry_1to2(pose_1, pose_2):
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
    T_w_r1 = pose_to_transformation_matrix(pose_1) # transformation matrix from world to robot11
    # print(T_w_r1)

    ### X_k -> T_w_r2 "robot r2 in world frame"
    T_w_r2 = pose_to_transformation_matrix(pose_2)
    # print(T_w_r2)

    # Get inverse of T_w_r1
    T_r1_w = np.linalg.inv(T_w_r1)
    # print("inverse", T_r1_w)

    # Multiply to get T_r1_r2
    T_r1_r2 = T_r1_w @ T_w_r2
    
    # Get pose from transformation matrix
    deltaX = transformation_matrix_to_pose(T_r1_r2)
    
    return deltaX  # [0, 0, 0]

def get_new_pose(pose_1, deltaX):

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
    T_w_r1 = pose_to_transformation_matrix(pose_1)
    T_r1_r2 = pose_to_transformation_matrix(deltaX)
    
    T_w_r2 = T_w_r1 @ T_r1_r2
    # print("T_w_r2", T_w_r2)
    pose_2 = transformation_matrix_to_pose(T_w_r2)  

    return pose_2
