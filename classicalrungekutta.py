#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The classical 4th order Runge-Kutta solver.

Takes a differential equation (or vector of equations) with an initial
condition and calculates the value of the function after a certain time step.

Routine Listings
-----------------
classical_RK4(x0, y0, dx, dy_dx)
Performs a 4th order Runge-Kutta operation on a given differential equation.

Notes
------
The classical Runge-Kutta method is a way to solve for the value of a function
y(t) after a specific time step h, given its derivative y_dot(t, y) and an
initial condition. It does this by estimating four increments (based on the
derivative at four different locations) and taking a weighted average of those
increments.

References
-----------
https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
https://www.myphysicslab.com/explain/runge-kutta-en.html
http://lpsa.swarthmore.edu/NumInt/NumIntFourth.html

Example
--------
Problem 7.17 in Phillips' Mechanics of Flight

import numpy as np
import classicalrungekutta as RK


def ODE(x0, variables):
    u = variables[0]
    w = variables[1]
    q = variables[2]
    t = variables[3]
    z = variables[4]
    th = variables[5]
    C = (1/(u*np.cos(th) + w*np.sin(th)))
    K0 = 0.00061  # 1/ft
    K1 = 0.14  # 1/ft
    K2 = 0.00059  # 1/ft
    K3 = 0.0016  # 1/ft^2
    K4 = 0.0064  # 1/ft
    g = 32.2  # ft/s^2
    ODE = np.zeros(6).T
    ODE[0] = C*(-K0*(u**2) - K1*(w**2) - g*np.sin(th) - q*w)
    ODE[1] = C*(-K2*u*w + g*np.cos(th) + q*u)
    ODE[2] = C*(-K3*u*w - K4*q*u)
    ODE[3] = C*1.0
    ODE[4] = C*(w*np.cos(th) - u*np.sin(th))
    ODE[5] = C*q
    return ODE


Vo = 210  # ft/s
theta0 = 5.0  # deg
w0 = 0.0
q0 = 0.0
t0 = 0.0
zf0 = 0.0
degtorad = 57.2957795
h = 3.  # ft
xmax = 300.  # ft
xf = np.arange(0.0, xmax + h, h)
N = len(xf)
variables0 = np.array([Vo, w0, q0, t0, zf0, theta0/degtorad]).T
print("Time\tRange\tElevation\tSpeed\ttheta\talpha")
for i in range(N):
    guess = RK.classical_RK4(xf[i], variables0, h, ODE)
    alpha = np.arctan(guess[1]/guess[0])
    V = np.sqrt(guess[0]**2 + guess[1]**2)
    variables0 = np.copy(guess)
    print(guess[3], "\t", xf[i], "\t", -12*guess[4], "\t", V, "\t",
          degtorad*guess[5], "\t", degtorad*alpha)

"""


def classical_RK4(x0, y0, dx, dy_dx):
    k1 = dy_dx(x0, y0)
    x1 = x0 + k1*(dx/2.0)
    y1 = y0 + k1*(dx/2.0)
    k2 = dy_dx(x1, y1)
    x2 = x0 + k2*(dx/2.0)
    y2 = y0 + k2*(dx/2.0)
    k3 = dy_dx(x2, y2)
    x3 = x0 + k3*dx
    y3 = y0 + k3*dx
    k4 = dy_dx(x3, y3)
    yp1 = y0 + (dx/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
    return yp1
