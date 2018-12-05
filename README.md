Robot Controller Service
==================

Fuse the robot pose measured by all the available cameras and use it to compute the velocity commands that need to be sent to the robot in order to execute a given task.

Dependencies:
-----
This service depends on the following services:

* [is-broker-events](https://github.com/labviros/is-broker-events): Used to check which cameras are available.
* [is-frame-transformation](https://github.com/labviros/is-frame-transformation): Used grab the transformations from the robot frame to the world frame using the available cameras. Note that the transformations should be available somehow, this may imply a dependency on other services like [is-aruco-detector](https://github.com/labviros/is-aruco-detector), for instance.
* [is-robots](https://github.com/labviros/is-robots): This service expects the robot to have a gateway compliant with the standard API. Particularly, it expects the robot to have **RobotGateway.{robot_id}.SetConfig** implemented.


Events:
--------
<img width=1000/> ⇒ Triggered By | <img width=1000/> Triggers ⇒ | <img width=200/> Description  
:------------ | :-------- | :----------
:incoming_envelope: **topic:** `RobotController.{robot_id}.SetTask` <br> :gem: **schema:** [RobotTaskRequest] | :incoming_envelope: **topic:** `{request.reply_to}` <br> :gem: **schema:** [RobotTaskReply] | `Configure the current task to be executed.`
:clock5: **interval:** `{RobotTask.rate}` | :incoming_envelope: **topic:** `RobotController.{robot_id}.Status` <br> :gem: **schema:** [RobotControllerProgress] | `Periodically publishes the progress of the current task being executed. The period is determined by the task sampling rate.`


[RobotTaskRequest]: https://github.com/labviros/is-msgs/tree/master/docs#is.robot.RobotTaskRequest
[RobotTaskReply]: https://github.com/labviros/is-msgs/tree/master/docs#is.robot.RobotTaskReply
[RobotControllerProgress]: https://github.com/labviros/is-msgs/tree/master/docs#is.robot.RobotControllerProgress


Configuration:
----------------
The behavior of the service can be customized by passing a JSON configuration file as the first argument, e.g: `./service config.json`. The schema and documentation for this file can be found in [`src/is/robot-controller/conf/options.proto`](src/is/robot-controller/conf/options.proto). An example configuration file can be found in [`etc/conf/options.json`](etc/conf/options.json).


Examples:
------------
An example on how to configure a task and then watch its progress is provided by the python script in [`examples/client.py`](examples/client.py).