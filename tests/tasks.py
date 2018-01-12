from sympy import *
import numpy as np

from robot_pb2 import RobotTask, TrajectoryTask, RobotControllerProgress
from common_pb2 import SamplingSettings, Pose, Position, Speed
from robot_parameters_pb2 import Parameters

def eight_trajectory(Ax, Ay, X0, Y0, tf, rate, stop_distance):
    t = Symbol('t')
    w = Symbol('w')
    phi = Symbol('phi')
    x0 = Symbol('x0')
    y0 = Symbol('y0')

    ax = (2 * Ax) / (3 - cos(2 * (w * t + phi)))
    ay = (Ay / 0.35) / (3 - cos(2 * (w * t + phi)))

    x = ax * cos(w * t + phi) / 2 + x0
    y = ay * sin(2 * (w * t + phi)) / 2 + y0
    dx = diff(x, t)
    dy = diff(y, t)

    _X = lambdify((w, t, phi, x0), x, 'numpy')
    _Y = lambdify((w, t, phi, y0), y, 'numpy')
    _dX = lambdify((w, t, phi, x0), dx, 'numpy')
    _dY = lambdify((w, t, phi, y0), dy, 'numpy')

    T = 1 / rate
    t = np.arange(0, tf, T)
    w = 2 * np.pi / tf
    phi = np.pi / 3

    X = _X(w, t, phi, X0)
    Y = _Y(w, t, phi, Y0)
    dX = _dX(w, t, phi, X0)
    dY = _dY(w, t, phi, Y0)

    positions = np.transpose(np.concatenate(
        (np.expand_dims(X, axis=1), np.expand_dims(Y, axis=1)),
        axis=1))
    speeds = np.transpose(np.concatenate(
        (np.expand_dims(dX, axis=1), np.expand_dims(dY, axis=1)),
        axis=1))

    pb_positions = [Position(x=positions[0, i], y=positions[1, i])
                     for i in range(positions.shape[1])]
    pb_speeds = [Speed(linear=speeds[0, i], angular=speeds[1, i])
                  for i in range(speeds.shape[1])]

    trajectory = RobotTask(
        trajectory=TrajectoryTask(positions=pb_positions, speeds=pb_speeds),
        allowed_error=stop_distance,
        sampling=SamplingSettings(frequency=rate))

    return trajectory