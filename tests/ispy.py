import pika
import uuid
import robot_pb2
from datetime import datetime
import json

def current_time():
    return int((datetime.utcnow() - datetime(1970, 1, 1)).total_seconds() * 1000)


def pack(message):
    body = message.SerializeToString()
    content_type = 'application/x-protobuf'
    return body, content_type


class Connection:
    def __init__(self, hostname, port):
        params = pika.ConnectionParameters(hostname, port)
        self.connection = pika.BlockingConnection(params)
        self.channel = self.connection.channel()
        ok = self.channel.queue_declare(exclusive=True, durable=False)
        self.exchange = 'is'
        self.queue = ok.method.queue
        self.channel.queue_bind(
            self.queue, self.exchange, routing_key=self.queue)
        self.subscriptions = {}
        self.rpcs = {}


    def subscribe(self, topic, pb, fn):
        def wrapped(c, context, body):
            msg = pb()
            msg.ParseFromString(body)
            fn(c, context, msg)
        self.channel.queue_bind(self.queue, self.exchange, topic)
        self.subscriptions[topic] = wrapped


    def rpc(self, topic, rep, fn):
        def stub(req):
            def callback(c, context, body):
                msg = rep()
                msg.ParseFromString(body)
                fn(c, context, msg)
            return self.request(topic, req, callback)
        return stub


    def request(self, endpoint, message, callback):
        cid = str(uuid.uuid4())
        body, content_type = pack(message)
        props = pika.BasicProperties(
            reply_to=self.queue, correlation_id=cid, content_type=content_type, timestamp=current_time())
        self.channel.basic_publish(self.exchange, endpoint, body, props)
        self.rpcs[cid] = callback
        return cid


    def publish(self, topic, message):
        body, content_type = pack(message)
        props = pika.BasicProperties(
            content_type=content_type, timestamp=current_time())
        self.channel.basic_publish(self.exchange, topic, body, props)

    def listen(self):

        def on_message(channel, method, props, body):
            rkey = method.routing_key
            cid = props.correlation_id
            context = {'timestamp': props.timestamp,
                        'headers': props.headers}
            if rkey in self.subscriptions:
                self.subscriptions[rkey](self, context, body)
            elif rkey == self.queue:
                rpc_status = context['headers']['rpc-status']
                context['headers']['rpc-status'] = json.loads(rpc_status)
                self.rpcs[cid](self, context, body)

        self.channel.basic_consume(on_message, queue=self.queue, no_ack=True)
        self.channel.start_consuming()


# def save_tracing(robot_status, exp_date):
#     begin = robot_status[0]['timestamp'] / 1000
#     end = robot_status[-1]['timestamp'] / 1000
#     trace_endpoint = 'http://192.168.1.100:30200/zipkin/api/v1/traces?' + \
#         'endTs={}&lookback={}&limit={}&minDuration=&serviceName=robotgateway.0&sortOrder=timestamp-desc'.format(
#             end, end - begin, len(robot_status))
#     filename = exp_date + '_tracing.json'
    #  with open(filename, 'w') as outfile:

#         json.dump(json.loads(tracing), outfile, indent=4)
#     print 'Tracing file saved on {}'.format(filename)


# def save_status(robot_status, exp_date):
#     filename = exp_date + '_robot_status.json'
#     with open(filename, 'w') as outfile:
#         json.dump(robot_status, outfile, indent=4)
#     print 'Robot status file saved on {}'.format(filename)


# def on_message(ch, method, props, body, robot_status, queue_name):
#     if method.routing_key == 'RobotController.0.Status':
#         status = robot_pb2.RobotControllerProgress()
#         status.ParseFromString(body)
#         status_dict = json.loads(MessageToJson(status, True))
#         robot_status.append(
#             {'timestamp': 1000 * props.timestamp, 'status': status_dict})
#         if status.done:
#             exp_date = time.strftime('%y-%m-%d_%H-%M')
#             save_status(robot_status, exp_date)
#             save_tracing(robot_status, exp_date)
#             sys.exit(0)
#     else:
#         print 'RobotController.0.SetTask status: {}'.format(props.headers['rpc-status'])
#         ch.queue_bind(exchange='is', queue=queue_name,
#                       routing_key='RobotController.0.Status')


# def request(endpoint, task):
#     connection = pika.BlockingConnection(
#         pika.ConnectionParameters('rmq.is', 30000))
#     channel = connection.channel()

#     result = channel.queue_declare(exclusive=True, durable=False)
#     queue_name = result.method.queue

#     channel.queue_bind(exchange='is', queue=queue_name, routing_key=queue_name)

#     cid = str(uuid.uuid4())
#     props = pika.BasicProperties(reply_to=queue_name, correlation_id=cid)
#     channel.basic_publish(exchange='is', routing_key='RobotController.0.SetTask',
#                           properties=props, body=json.dumps(task))

#     robot_status = []
#     channel.basic_consume(lambda c, m, p, b: on_message(
#         c, m, p, b, robot_status, queue_name), queue=queue_name, no_ack=True)
#     channel.start_consuming()
