#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np
import math
from math import sin,cos
import motor_command_model as mcm

# Closed form solution parameters
t = np.linspace(0,math.pi,num=50)

# r = 1.0
# omega = 1.0
# cx = 0.0
# cy = 0.0
# ct = 1.57

sl = 0.5
sr = 1.0
u = np.array([[sl],[sr]])
z = np.array([[0.0],[0.0],[0.0]])
##### Compute plot values #####
# Moving in an arc
if abs(float(u[0]-u[1])) > float(math.pow(10,-3)):
        r,omega,cx,cy,ct = mcm.closed_form_parameters(z,u)

        x = r * np.sin(omega*t + ct) + cx
        y = -r * np.cos(omega*t + ct) + cy
        theta = omega*t + ct
        label = 1

# Straight line motion
else:
    k,d = mcm.model_parameters()
    sl = u[0]
    sr = u[1]

    x = (k/2.0)*cos(z[2])*(sl+sr)*t + z[0]
    y = (k/2.0)*sin(z[2])*(sl+sr)*t + z[1]
    theta = z[2]
    label = 0

# Plot values
plt.plot(x,y)
pltmin = -10
pltmax = 10

# Plot labels
plt.xlabel("x")
plt.ylabel("y")
plt.title('Varying Parameters to Closed Form Solution')
if label:
    s = 'omega = {}\nr = {}\ncx = {}\ncy = {}\nct = {}'.format(omega,r,cx,cy,ct)
    font = {'family': 'serif',
            'color':  'blue',
            'weight': 'normal',
            'size': 12,
            }
    plt.text(pltmax+.15, 1, s, fontdict = font)


# Set axis limits and force equal aspect ratio
plt.xlim(pltmin, pltmax)
plt.ylim(pltmin, pltmax)
plt.gca().set_aspect('equal', adjustable='box')
plt.grid(color='k', linestyle='-', linewidth=.5)

# Display plot
plt.show()