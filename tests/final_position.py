#!/usr/bin/env python

import argparse, json, sys
import ispy, saver
from robot_pb2 import RobotTask, FinalPoseTask, RobotControllerProgress
from common_pb2 import SamplingSettings, Pose, Position
from google.protobuf.empty_pb2 import Empty
from google.protobuf.json_format import MessageToJson


def main():
    parser = argparse.ArgumentParser(description='Final position test.')
    parser.add_argument('-x', type=float, default=0.0)
    parser.add_argument('-y', type=float, default=0.0)
    parser.add_argument('-r', type=float, default=5.0)
    parser.add_argument('-e', type=float, default=0.1)
    options = parser.parse_args()

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

    def on_reply(c, context, msg):
        print context['headers']['rpc-status']
        c.subscribe('RobotController.0.Status', RobotControllerProgress, on_status)

    set_task = c.rpc('RobotController.0.SetTask', Empty, on_reply)

    final_position = RobotTask(
        pose = FinalPoseTask(goal = Pose(position = Position(x = options.x, y = options.y))), 
        allowed_error = options.e,
        sampling = SamplingSettings(frequency = options.r))

    set_task(final_position)

    c.listen()

if __name__ == "__main__":
    main()