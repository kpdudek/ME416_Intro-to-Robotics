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
    if abs(lin) > 0 and abs(ang) > 0:
        left = 1.0 * lin
        right = 1.0 * lin

        if ang > 0:
            left = left * (1-ang)
        elif ang < 0:
            right = right * (1+ang)
    elif abs(lin) < 0.05 and abs(ang) > 0:
        if ang > 0:
            left = -1+ang
            right = 1-ang
        elif ang < 0:
            right = -1-ang
            left = 1+ang
    else:
        left = 1.0 * lin
        right = 1.0 * lin


    return left,right
