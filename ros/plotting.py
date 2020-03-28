#!/usr/bin/env python

from matplotlib import pyplot as plt
import numpy as np
import math #needed for definition of pi
from math import sin,cos

# Closed form solution parameters
t = np.linspace(0,math.pi,num=50)
# r = 1.25
# omega = 1
# cx = 0
# cy = 0
# ct = 0

k = 4.0
d = 0.5
sl = 1.0
sr = 0.5

z_zero = np.array([[0.0],[0.0],[0.0]])
r = (d*(sl+sr))/(sl-sr)
omega = (k/2.0*d) * (sr-sl)
cx = z_zero[0] - r*np.sin(z_zero[2]) 
cy = z_zero[1] + r*np.cos(z_zero[2]) 
ct = z_zero[2]

##### Compute plot values #####
### Arc
x = r * np.sin(omega*t) + cx
y = -r * np.cos(omega*t) + cy
theta = omega*t + ct

### Straight line
# x = (k/2.0)*cos(z_zero[2])*(sl+sr)*t + z_zero[0]
# y = (k/2.0)*sin(z_zero[2])*(sl+sr)*t + z_zero[1]
# theta = ct = z_zero[2]


# Plot values
plt.plot(x,y)

# Plot labels
plt.xlabel("x")
plt.ylabel("y")
plt.title('Varying Parameters to Closed Form Solution')

# Set axis limits and force equal aspect ratio
pltmin = -2
pltmax = 2
plt.xlim(pltmin, pltmax)
plt.ylim(pltmin, pltmax)
plt.gca().set_aspect('equal', adjustable='box')

# Display plot
plt.show()