#!/usr/bin/env python

import argparse, json, sys
import ispy, saver, tasks
from robot_pb2 import RobotControllerProgress
from robot_parameters_pb2 import Parameters
from google.protobuf.empty_pb2 import Empty
from google.protobuf.json_format import MessageToJson, Parse


def main():
    parser = argparse.ArgumentParser(
        description='Generates positions and speeds.')
    parser.add_argument('-d', nargs=2, type=float, default=[4.0, 1.2])
    parser.add_argument('-o', nargs=2, type=float, default=[0.0, 0.0])
    parser.add_argument('-t', type=float, default=25.0)
    parser.add_argument('-r', type=float, default=5.0)
    parser.add_argument('-e', type=float, default=0.1)
    parser.add_argument('-p', default='../parameters.json')
    options = parser.parse_args()

    with open(options.p, 'r') as par_file:
        parameters = Parse(par_file.read(), Parameters())

    Ax, Ay = options.d[0], options.d[1]
    X0, Y0 = options.o[0], options.o[1]
    trajectory = tasks.eight_trajectory(Ax, Ay, X0, Y0, options.t, options.r, options.e)

    c = ispy.Connection('rmq.is', 30000)
    robot_status = []

    def on_status(c, context, msg):
        status_dict = json.loads(MessageToJson(msg, True))
        robot_status.append(
            {'timestamp': 1000 * context['timestamp'], 'status': status_dict}
        )
        if msg.done:
            exp_date = saver.now()
            saver.status(robot_status, exp_date)
            saver.tracing(robot_status, exp_date)
            sys.exit(0)

    def task_reply(c, context, msg):
        print 'Task reply: {}'.format(context['headers']['rpc-status'])
        c.subscribe('RobotController.0.Status',
                    RobotControllerProgress, on_status)

    def parameters_reply(c, context, msg):
        print 'Parameters reply: {}'.format(context['headers']['rpc-status'])
        set_task(trajectory)

    set_task = c.rpc('RobotController.0.SetTask', Empty, task_reply)
    set_parameters = c.rpc('RobotController.0.SetParameters', Empty, parameters_reply)

    set_parameters(parameters)
    c.listen()

if __name__ == "__main__":
    main()