"""Functions for modeling ROSBot"""

import numpy as np
from math import cos, sin

def model_parameters():
    """Returns two constant model parameters"""
    k = 1.0
    d = 0.5
    return k, d

def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    return A


def system_field(z, u):
    """Computes the field at a given state for the dynamical model"""
    return dot_z


def euler_step(z, u, stepSize):
    """Integrates the dynamical model for one time step using Euler's method"""
    return zp

def twist_to_speeds(lin,ang):
    """This function is passed the desired robot linear and angular speeds and returns the individual motor commands"""
    anglular_offset = 1.1

    if lin > 0.0 and abs(ang) > 0.0:
        left = 1.0 * lin
        right = 1.0 * lin

        if ang > 0.0:
            left = left * (anglular_offset-ang)
        elif ang < 0.0:
            right = right * (anglular_offset+ang)
    
    elif lin < 0.0 and abs(ang) > 0.0:
        left = 1.0 * lin
        right = 1.0 * lin

        if ang > 0.0:
            right = right * (anglular_offset-ang)
        elif ang < 0.0:
            left = left * (anglular_offset+ang)

    elif abs(lin) == 0.0 and abs(ang) > 0.0:
        if ang > 0.0:
            left = -anglular_offset+ang
            right = anglular_offset-ang
        elif ang < 0.0:
            right = -anglular_offset-ang
            left = anglular_offset+ang

    else:
        left = 1.0 * lin
        right = 1.0 * lin


    return left,right
