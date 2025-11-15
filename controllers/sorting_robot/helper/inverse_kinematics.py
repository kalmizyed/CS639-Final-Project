import numpy as np
import math
from scipy.spatial.transform import Rotation as R # you might find this useful for calculating the error
from helper.ur5e_transformations import *

### Tuning
E_MAX = 0.01
I_MAX = 10
DAMP = 0.000001

# Get the inverse kinematics solution for the UR5E given the desired pose in the
# task space and its current pose in the configuration space.
def getInverseKinematics(desired_pose: list[float], current_q: list[float]) -> list[float]:
    # desired_pose: a list of six for the desired position (3) and desired
    # exponential coordinates (3)

    # current q is the list of six joint angles: initially the current six joint
    # angles, and then the last commanded joint angles after that

    # returns a six-element list of the joint angles in radians

    # Tuning
    e_max = E_MAX
    i_max = I_MAX
    damp = DAMP
    initial_guess = current_q

    # Initial guess
    i = 0
    theta_i = np.array(initial_guess, dtype=np.float64)
    e = getError(desired_pose, getFK(theta_i))

    # Iterate until error is sufficiently small
    while np.linalg.norm(e) > e_max and i < i_max:
        jacobian = getJacobian(theta_i)
        # Pseudoinverse method with damping to avoid singularities
        inv_Jacobian = np.matmul(jacobian.T, np.linalg.inv(np.matmul(jacobian, jacobian.T)))
        theta_i = theta_i + np.matmul(inv_Jacobian, e)
        i += 1
        e = getError(desired_pose, getFK(theta_i))

    # Return resulting joint angles
    return list(theta_i)

# Generate the transformation matrix from the base frame to the desired joint
# frame for the robot at its current joint angles q.
def getTransformTo(joint: int, q: list[float]) -> list[list[float]]:
    T = [T01, T12, T23, T34, T45, T56]
    out = np.identity(4)
    for i in range(joint):
        out = np.matmul(out, T[i](q[i]))
    return out

# Generate the forward-kinematics matrix for the robot at its
# current joint angles q.
def getFK(q: list[float]) -> list[list[float]]:
    return getTransformTo(6, q)

# Given a desired pose and current guess pose, calculate the error for
# the Newton-Raphson algorithm.
def getError(pose_d: list[float], fk: list[list[float]]) -> list[float]:
    pose_q = fk

    # Get translation error
    x_d = np.array(pose_d[:3], dtype=np.float64)
    x_q = np.array(pose_q[:3, 3], dtype=np.float64)
    e_pos = x_d - x_q

    # Get rotation error
    R_d = R.from_rotvec(pose_d[3:])
    R_q = R.from_matrix(pose_q[:3,:3])
    e_rot = np.array((R_d * R_q.inv()).as_rotvec(), dtype=np.float64)
    return np.concatenate([e_pos, e_rot])

def isPoseAcceptable(pose_d: list[float], joints: list[float]) -> bool:
    error = getError(pose_d, getFK(joints))
    return np.linalg.norm(error) <= E_MAX

# Given a set of joint angles q, generate the Jacobian matrix relating joint
# velocities to task-space velocities.
def getJacobian(q: list[float]) -> list[list[float]]:
    q = np.asarray(q, dtype=np.float64)
    J = np.zeros((6, 6), dtype=np.float64)

    transforms = [T01, T12, T23, T34, T45, T56]

    # Get transforms T_0i for 1<i<6
    T = [np.eye(4)]
    for i in range(6):
        T_i = T[i] @ transforms[i](q[i])
        T.append(T_i)
    
    # Get FK
    p_e = getTransformTo(6, q)[:3, 3]

    for i in range(6):
        z_i = T[i][:3, 2]
        p_i = T[i][:3, 3]
        J_vi = np.cross(z_i, (p_e - p_i))
        J_wi = z_i
        J[0:3, i] = J_vi
        J[3:6, i] = J_wi
    
    return np.array(J, dtype=np.float64)
