import pika
import uuid
import json
import sys


def rpc_reply(ch, method, props, body):
    print props.headers['rpc-status']
    sys.exit(0)


def request(task):
    connection = pika.BlockingConnection(pika.ConnectionParameters('rmq.is', 30000))
    channel = connection.channel()

    result = channel.queue_declare(exclusive=True, durable=False)
    queue_name = result.method.queue

    channel.queue_bind(exchange='is', queue=queue_name, routing_key=queue_name)

    cid = str(uuid.uuid4())
    props = pika.BasicProperties(reply_to=queue_name, correlation_id=cid)
    channel.basic_publish(exchange='is', routing_key='RobotController.0.SetTask',
                            properties=props, body=json.dumps(task))

    channel.basic_consume(rpc_reply, queue=queue_name, no_ack=True)
    channel.start_consuming()