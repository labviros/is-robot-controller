#!/usr/bin/env python
from sympy import *
import numpy as np
import argparse
import task
import matplotlib.pyplot as plt

def planning_circle(X0, Y0, R, tf, rate, stop_distance) :
    t = Symbol('t')
    w = Symbol('w')
    phi = Symbol('phi')
    x0 = Symbol('x0')
    y0 = Symbol('y0')

    x = R*cos(w*t + phi) + x0
    y = R*sin(w*t + phi) + y0
    dx = diff(x, t)
    dy = diff(y, t)

    _X = lambdify((w, t, phi, x0), x, 'numpy')
    _Y = lambdify((w, t, phi, y0), y, 'numpy')
    _dX = lambdify((w, t, phi, x0), dx, 'numpy')
    _dY = lambdify((w, t, phi, y0), dy, 'numpy')

    T =  1/rate
    t = np.arange(0, tf, T)
    w = 2*np.pi/tf
    # phi = np.pi/3
    phi = 0.0

    X = _X(w, t, phi, X0)
    Y = _Y(w, t, phi, Y0)
    dX = _dX(w, t, phi, X0)
    dY = _dY(w, t, phi, Y0)

    # plt.figure()
    # plt.plot(X, Y)

    # plt.figure()
    # plt.plot(t, np.sqrt(np.abs(dX*dX) + np.abs(dY*dY)))

    # plt.show()

    positions = np.transpose(np.concatenate(
        (np.expand_dims(X, axis=1), np.expand_dims(Y, axis=1)), 
        axis = 1))
    speeds = np.transpose(np.concatenate(
        (np.expand_dims(dX, axis=1), np.expand_dims(dY, axis=1)), 
        axis = 1))

    ret_positions = [{"x": positions[0,i], "y":positions[1,i]} for i in range(positions.shape[1])]
    ret_speeds = [{"linear": speeds[0,i], "angular":speeds[1,i]} for i in range(speeds.shape[1])]

    return (ret_positions, ret_speeds)

def main() :
    parser = argparse.ArgumentParser(description='Generates positions and speeds.')
    parser.add_argument('-R', type=float)
    parser.add_argument('-o', nargs=2, type=float)
    parser.add_argument('-t', type=float)
    parser.add_argument('-r', type=float, default=5.0)
    parser.add_argument('-e', type=float, default=0.1)

    options = parser.parse_args()
    
    R = options.R
    X0 = options.o[0]
    Y0 = options.o[1]
    
    positions, speeds = planning_circle(X0, Y0, R, options.t, options.r, options.e)

    trajectory = {
        "trajectory": {"positions": positions, "speeds": speeds}, "allowed_error": options.e,
        "sampling": {"frequency": options.r}
    }

    task.request(trajectory)

    

if __name__ == "__main__":
    main()
