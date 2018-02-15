#!/usr/bin/env python
import argparse
import json
import sys
import os
import ispy
import tasks
from ismsgs.robot_parameters_pb2 import Parameters
from google.protobuf.json_format import MessageToJson, Parse

parser = argparse.ArgumentParser(description='Requests a task.')
parser.add_argument('-f', type=str, default='task.json',
                    help='JSON file with trajectory parameters and optional robot parameters.')
parser.add_argument('-b', type=str, default='localhost:5672', help='AMQP Broker hostname.')
parser.add_argument('-z', type=str, default='localhost:9411', help='Zipkin hostname.')
parser.add_argument('-o', type=str, default='./', help='Results folder')
options = parser.parse_args()

# read parameters files
with open(options.f, 'r') as task_file:
    task_parameters = json.load(task_file)
with open('../parameters.json', 'r') as parameters_file:
    robot_parameters = Parse(parameters_file.read(), Parameters())

# create output folder
output_folder = options.o
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# common parameters
rate = task_parameters['sampling_rate']
allowed_error = task_parameters['allowed_error']
if task_parameters['type'] == 'eight_trajectory':
    Ax, Ay = task_parameters['shape']['x-axis'], task_parameters['shape']['y-axis']
    X0, Y0 = task_parameters['center']['x'], task_parameters['center']['y']
    tf = task_parameters['lap_time']
    laps = task_parameters['laps']
    task = tasks.eight_trajectory(Ax, Ay, X0, Y0, tf, rate, allowed_error, laps)
elif task_parameters['type'] == 'circle_trajectory':
    X0, Y0 = task_parameters['center']['x'], task_parameters['center']['y']
    R = task_parameters['shape']['radius']
    tf = task_parameters['lap_time']
    laps = task_parameters['laps']
    task = tasks.eight_trajectory(Ax, Ay, R, tf, rate, allowed_error, laps)
elif task_parameters['type'] == 'final_position':
    X, Y = task_parameters['goal']['x'], task_parameters['goal']['y']
    task = tasks.final_position(X, Y, allowed_error, rate)
else:
    print 'Invalid task type on file {}. Exiting.'.format(options.f)

# parse and marge robot parameters
if 'robot_parameters' in task_parameters.keys():
    robot_parameters = Parse(json.dumps(task_parameters['robot_parameters']), Parameters())

zipkin_hostname = options.z
broker_hostname = options.b.split(':')
ip, port = broker_hostname[0], int(broker_hostname[1])
c = ispy.Connection(ip, port)

tasks.request_and_save(c, task, robot_parameters, output_folder, zipkin_hostname)
c.listen()