import json, time, requests

def tracing(robot_status, exp_date):
    begin = robot_status[0]['timestamp'] / 1000
    end = robot_status[-1]['timestamp'] / 1000
    trace_endpoint = 'http://192.168.1.100:30200/zipkin/api/v1/traces?' + \
        'endTs={}&lookback={}&limit={}&minDuration=&serviceName=robotgateway.0&sortOrder=timestamp-desc'.format(
            end, end - begin, len(robot_status))
    tracing = requests.get(trace_endpoint).text
    filename = exp_date + '_tracing.json'
    with open(filename, 'w') as outfile:
        json.dump(json.loads(tracing), outfile, indent=4)
    print 'Tracing file saved on {}'.format(filename)


def status(robot_status, exp_date):
    filename = exp_date + '_robot_status.json'
    with open(filename, 'w') as outfile:
        json.dump(robot_status, outfile, indent=4)
    print 'Robot status file saved on {}'.format(filename)


def now():
    return time.strftime('%y-%m-%d_%H-%M')