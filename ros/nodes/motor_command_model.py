"""Functions for modeling ROSBot"""

import rospy
import numpy as np
from math import cos, sin

def model_parameters():
    """Returns two constant model parameters"""
    k = 1.0
    d = 0.5
    return k, d

def closed_form_parameters(z_zero,u):
    k,d = model_parameters()
    sl = u[0]
    sr = u[1]

    r = (d*(sl+sr))/(sl-sr)
    omega = (k/2*d) * (sr-sl)
    c_x = z_zero[0] - r*sin(z_zero[2]) 
    c_y = z_zero[1] - r*cos(z_zero[2]) 
    c_theta = z_zero[2]

    return r,omega,c_x,c_y,c_theta

def closed_form_step(z,u,T):
    if T == None:
        return z
    
    if abs(u[0]-u[1]) < np.pow(10,-3):
        r,omega,c_x,c_y,c_theta = closed_form_parameters(z,u)

        x = r * np.sin(omega*T) + cx
        y = -r * np.cos(omega*T) + cy
        theta = omega*T + ct
    else:
        k,d = model_parameters()
        sl = u[0]
        sr = u[1]
        x = (k/2.0)*cos(z[2])*(sl+sr)*T + z[0]
        y = (k/2.0)*sin(z[2])*(sl+sr)*T + z[1]
        theta = ct = z[2]

    zp = np.array([[x],[y],[theta]])
    return zp

def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    k,d = model_parameters()
    A = (k/2.0) * np.array([[cos(theta), cos(theta)],[sin(theta), sin(theta)],[-1.0/d, 1.0/d]])
    return A


def system_field(z, u):
    """Computes the field at a given state for the dynamical model"""
    return dot_z


def euler_step(z, u, stepSize):
    """Integrates the dynamical model for one time step using Euler's method"""
    A = system_matrix(z[2])
    zp = z + stepSize*A.dot(u)
    return zp

def twist_to_speeds(lin,ang):
    """This function is passed the desired robot linear and angular speeds and returns the individual motor commands"""
    k,d = model_parameters()

    left = (lin - d*ang) / k 
    right = (lin + d*ang) / k

    if abs(left) > 1.0:
        left = np.sign(left) * 1.0
    if abs(right) > 1.0:
        right = np.sign(right) * 1.0

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

class StampedMsgRegister(object):
    def __init__(self):
        self.msg_previous = None

    def replace_and_compute_delay(self,msg):
        if self.msg_previous == None:
            msg_previous = None
            self.msg_previous = msg
            time_delay = None
        else:
            msg_previous = self.msg_previous
            self.msg_previous = msg
            time_delay = msg.header.stamp.to_sec() - msg_previous.header.stamp.to_sec()

        return time_delay, msg_previous

