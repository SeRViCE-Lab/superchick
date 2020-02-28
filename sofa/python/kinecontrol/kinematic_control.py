import numpy as np
import math

def get_position_vector(cartesian_coords):
    '''
    Given the coordinates of a point on a
    soft actuator's body, compute the equivalent
    spherical coordinates. See equation 9 and 77
    in my kinecontrol paper

        cartesian coords is N by (x,y,z)
        where N is the length of the data
    '''
    x = np.array([t[0] for t in cartesian_coords])
    y = np.array([t[1] for t in cartesian_coords])
    z = np.array([t[2] for t in cartesian_coords])

    x_sq = np.array([np.square(t[0]) for t in x])
    y_sq = np.array([np.square(t[1]) for t in y])
    z_sq = np.array([np.square(t[2]) for t in z])

    # spherical coordinates equivalent
    r = np.sqrt(x_sq+ y_sq+z_sq)
    theta = np.arctan(y/x)
    phi = np.np.arccos(z/r)

    return r, theta, phi
