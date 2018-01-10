import pika
import uuid
import json
import sys
import robot_pb2
from google.protobuf.json_format import MessageToJson
import datetime
import requests


def save_tracing(robot_status, exp_date):
    begin = robot_status[0]['timestamp'] / 1000
    end = robot_status[-1]['timestamp'] / 1000
    trace_endpoint = 'http://192.168.1.100:30200/zipkin/api/v1/traces?' + \
        'endTs={}&lookback={}&limit={}&minDuration=&serviceName=robotgateway.0&sortOrder=timestamp-desc'.format(
           end, end - begin, len(robot_status))
    tracing = requests.get(trace_endpoint).text
    filename = exp_date + 'tracing.json'
    with open(filename, 'w') as outfile:
        json.dump(json.loads(tracing), outfile, indent=4)


def save_status(robot_status, exp_date):
    filename = exp_date + 'robot_status.json'
    with open(filename, 'w') as outfile:
        json.dump(robot_status, outfile, indent=4)


def on_message(ch, method, props, body, robot_status, queue_name):
    if method.routing_key == 'RobotController.0.Status':
        status = robot_pb2.RobotControllerProgress()
        status.ParseFromString(body)
        status_dict = json.loads(MessageToJson(status, True))
        robot_status.append(
            {'timestamp': 1000 * props.timestamp, 'status': status_dict})
        if status.done:
            exp_date = str(datetime.datetime.now())
            save_status(robot_status, exp_date)
            save_tracing(robot_status, exp_date)
            sys.exit(0)
    else:
        print props.headers['rpc-status']
        ch.queue_bind(exchange='is', queue=queue_name,
                      routing_key='RobotController.0.Status')


def request(task):
    connection = pika.BlockingConnection(
        pika.ConnectionParameters('rmq.is', 30000))
    channel = connection.channel()

    result = channel.queue_declare(exclusive=True, durable=False)
    queue_name = result.method.queue

    channel.queue_bind(exchange='is', queue=queue_name, routing_key=queue_name)

    cid = str(uuid.uuid4())
    props = pika.BasicProperties(reply_to=queue_name, correlation_id=cid)
    channel.basic_publish(exchange='is', routing_key='RobotController.0.SetTask',
                          properties=props, body=json.dumps(task))

    robot_status = []
    channel.basic_consume(lambda c, m, p, b: on_message(
        c, m, p, b, robot_status, queue_name), queue=queue_name, no_ack=True)
    channel.start_consuming()
