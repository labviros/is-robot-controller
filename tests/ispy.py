import pika
import uuid
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