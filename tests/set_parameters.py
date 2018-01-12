#!/usr/bin/env python
import argparse
import pika
import uuid
import json
import sys

def on_reply(ch, method, props, body):
    print 'RobotController.0.SetParameters status: {}'.format(props.headers['rpc-status'])
    sys.exit(0)

def main() :
    parser = argparse.ArgumentParser()
    parser.add_argument('-f')
    options = parser.parse_args()

    filename = options.f if options.f is not None else '../parameters.json'
    with open(filename) as par_file:
        parameters = json.dumps(json.load(par_file))
    
    print parameters

    connection = pika.BlockingConnection(pika.ConnectionParameters('rmq.is', 30000))
    channel = connection.channel()
    
    result = channel.queue_declare(exclusive=True, durable=False)
    queue_name = result.method.queue
    channel.queue_bind(exchange='is', queue=queue_name, routing_key=queue_name)
    
    cid = str(uuid.uuid4())
    props = pika.BasicProperties(reply_to=queue_name, correlation_id=cid)
    channel.basic_publish(exchange='is', routing_key='RobotController.0.SetParameters',
                          properties=props, body=parameters)
    
    channel.basic_consume(on_reply, queue=queue_name, no_ack=True)
    channel.start_consuming()

if __name__ == "__main__":
    main()
