from sympy import *
import numpy as np
import json, sys, os, time, requests

from ismsgs.robot_pb2 import RobotTask, FinalPoseTask, TrajectoryTask, RobotControllerProgress
from ismsgs.common_pb2 import SamplingSettings, Pose, Position, Speed
from ismsgs.robot_parameters_pb2 import Parameters
from google.protobuf.empty_pb2 import Empty
from google.protobuf.json_format import MessageToJson


class Saver:

    def __init__(self, output_folder):
        self.time = time.strftime('%y-%m-%d_%H-%M')
        self.begin = 0
        self.end = 0
        self.samples = 0
        # create output folder
        self.output_folder = output_folder
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)
        filename = os.path.join(self.output_folder, 'info')
        with open(filename, 'w') as outfile:
            outfile.write(self.time)

    def robot_status(self, robot_status):
        filename = os.path.join(self.output_folder, 'robot_status.json')
        with open(filename, 'w') as outfile:
            json.dump(robot_status, outfile, indent=4)
            print 'Robot status file saved on {}'.format(filename)
        self.begin = robot_status[0]['timestamp'] / 1000
        self.end = robot_status[-1]['timestamp'] / 1000
        self.samples = len(robot_status)
    
    def tracing(self, zipkin_hostname):
        trace_endpoint = 'http://{}/zipkin/api/v1/traces?'.format(zipkin_hostname) + \
            'endTs={}&lookback={}&limit={}'.format(self.end, self.end - self.begin, self.samples) + \
            '&minDuration=&serviceName=robotgateway.0&sortOrder=timestamp-desc'
        time.sleep(3)
        tracing = requests.get(trace_endpoint).text
        filename = os.path.join(self.output_folder, 'tracing.json')
        with open(filename, 'w') as outfile:
            json.dump(json.loads(tracing), outfile, indent=4)
        print 'Tracing file saved on {}'.format(filename)


def pb_positions(X, Y):
    positions = np.transpose(np.concatenate(
        (np.expand_dims(X, axis=1), np.expand_dims(Y, axis=1)),
        axis=1))
    return [Position(x=positions[0, i], y=positions[1, i])
            for i in range(positions.shape[1])]


def pb_speeds(dX, dY):
    speeds = np.transpose(np.concatenate(
        (np.expand_dims(dX, axis=1), np.expand_dims(dY, axis=1)),
        axis=1))
    return [Speed(linear=speeds[0, i], angular=speeds[1, i])
            for i in range(speeds.shape[1])]


def make_trajectory(positions, speeds, allowed_error, sampling):
    return RobotTask(
        trajectory=TrajectoryTask(positions=positions, speeds=speeds),
        allowed_error=allowed_error,
        sampling=sampling)

def repeat_n(n, *args):
    return tuple([np.tile(arg, np.max([0, n])) for arg in args])

def eight_trajectory(Ax, Ay, X0, Y0, tf, rate, stop_distance, n=1):
    t, w, phi, x0, y0 = symbols('t,w,phi,x0,y0')

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

    X, Y = _X(w, t, phi, X0), _Y(w, t, phi, Y0)
    dX, dY = _dX(w, t, phi, X0), _dY(w, t, phi, Y0)
    X, Y, dX, dY = repeat_n(n, X, Y, dX, dY)

    return make_trajectory(pb_positions(X, Y), pb_speeds(dX, dY), stop_distance, SamplingSettings(frequency=rate))


def circle_trajectory(X0, Y0, R, tf, rate, stop_distance, n=1):
    t, w, phi, x0, y0 = symbols('t,w,phi,x0,y0')

    x = R * cos(w * t + phi) + x0
    y = R * sin(w * t + phi) + y0
    dx = diff(x, t)
    dy = diff(y, t)

    _X = lambdify((w, t, phi, x0), x, 'numpy')
    _Y = lambdify((w, t, phi, y0), y, 'numpy')
    _dX = lambdify((w, t, phi, x0), dx, 'numpy')
    _dY = lambdify((w, t, phi, y0), dy, 'numpy')

    T = 1 / rate
    t = np.arange(0, tf, T)
    w = 2 * np.pi / tf
    phi = 0.0

    X, Y = _X(w, t, phi, X0), _Y(w, t, phi, Y0)
    dX, dY = _dX(w, t, phi, X0), _dY(w, t, phi, Y0)
    X, Y, dX, dY = repeat_n(n, X, Y, dX, dY)

    return make_trajectory(pb_positions(X, Y), pb_speeds(dX, dY), stop_distance, SamplingSettings(frequency=rate))


def final_position(x, y, allowed_error, rate):
    return RobotTask(
        pose=FinalPoseTask(goal=Pose(position=Position(x=x, y=y))),
        allowed_error=allowed_error,
        sampling=SamplingSettings(frequency=rate))


def request_and_save(c, task, parameters, output_folder, zipkin_hostname):

    saver = Saver(output_folder)

    robot_status = []
    def on_status(c, context, msg):
        status_dict = json.loads(MessageToJson(msg, True))
        robot_status.append(
            {'timestamp': 1000 * context['timestamp'], 'status': status_dict}
        )
        if msg.done:
            saver.robot_status(robot_status)
            saver.tracing(zipkin_hostname)
            sys.exit(0)


    def task_reply(c, context, msg):
        print 'Task reply: {}'.format(context['headers']['rpc-status'])
        c.subscribe('RobotController.0.Status',
                    RobotControllerProgress, on_status)


    def parameters_reply(c, context, msg):
        print 'Parameters reply: {}'.format(context['headers']['rpc-status'])
        set_task(task)


    set_task = c.rpc('RobotController.0.SetTask', Empty, task_reply)
    set_parameters = c.rpc(
        'RobotController.0.SetParameters', Empty, parameters_reply)

    set_parameters(parameters)
    c.listen()