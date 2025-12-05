import math
import numpy as np

TIMESTEP_LENGTH = 0.032

# TODO implement a function to check for collisions along a given path and/or generate a collision-free path?

### Cubic Time Interpolation Functions ###

# Get the minimum number of timesteps required
# to move from start and end joint angles
def getRequiredTime(theta_start, theta_end):
    time = max([3 * abs(end - start) for start, end in zip(theta_start, theta_end)])
    return math.ceil(time / TIMESTEP_LENGTH)

# Get the desired position between start and end joint angles
# at timestep t out of maximum time steps T
def getPosition(theta_start, theta_end, t, T):
    t *= TIMESTEP_LENGTH
    T *= TIMESTEP_LENGTH
    start = np.array(theta_start)
    end = np.array(theta_end)

    interp = 3 * ((t/T)**2) - 2 * ((t/T)**3)
    pos = start + (interp * (end - start))
    return list(pos)
