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
    anglular_offset = 1.01
    float_tol = 0.01

    # Move forwards and arc
    if lin > float_tol and abs(ang) > float_tol:
        left = 1.0 * lin
        right = 1.0 * lin

        # Arc left
        if ang > 0.0:
            left = left * (anglular_offset-ang)
        # Arc right
        elif ang < 0.0:
            right = right * (anglular_offset+ang)
    
    # Move backwards and arc
    elif lin < -float_tol and abs(ang) > float_tol:
        left = 1.0 * lin
        right = 1.0 * lin

        # Arc left
        if ang > 0.0:
            right = right * (anglular_offset-ang)
        # Arc right
        elif ang < 0.0:
            left = left * (anglular_offset+ang)

    # Pivot in place
    elif abs(lin) <= float_tol and abs(ang) > float_tol:
        if ang > 0.0:
            left = -1*ang
            right = ang
        elif ang < 0.0:
            right = 1*ang
            left = -1*ang

    # Move robot straight
    else:
        left = lin
        right = lin

    return left,right

class KeysToVelocities(object):
    def __init__(self):
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.SPEED_DELTA = 0.2
        self.action = ''

    def update_speeds(self,key):
        if key == 'w' or key == 'W':
            self.speed_linear += self.SPEED_DELTA
            self.action = 'Increased Linear Speed by %f'%(self.SPEED_DELTA)
        elif key == 's' or key == 'S':
            self.speed_linear -= self.SPEED_DELTA
            self.action = 'Decreased Linear Speed by %f'%(self.SPEED_DELTA)
        elif key == 'a' or key == 'A':
            self.speed_angular += self.SPEED_DELTA
            self.action = 'Increased Angular Speed by %f'%(self.SPEED_DELTA)
        elif key == 'd' or key == 'D':
            self.speed_angular -= self.SPEED_DELTA
            self.action = 'Decreased Angular Speed by %f'%(self.SPEED_DELTA)
        elif key == 'z' or key == 'Z':
            self.speed_linear = 0.0
            self.action = 'Zeroed Linear Speed'
        elif key == 'c' or key == 'C':
            self.speed_angular = 0.0
            self.action = 'Zeroed Angular Speed'
        elif key == 'x' or key == 'X':
            self.speed_angular = 0.0
            self.speed_linear = 0.0
            self.action = 'Zeroed All Speeds'
        else:
            self.action = 'Invalid Key Press'

        # Ensure published value is in range [-1,1]
        if self.speed_angular > 1.0:
            self.speed_angular = 1.0
        elif self.speed_angular < -1.0:
            self.speed_angular = -1.0

        if self.speed_linear > 1.0:
            self.speed_linear = 1.0
        elif self.speed_linear < -1.0:
            self.speed_linear = -1.0            
        
        return self.speed_linear, self.speed_angular
