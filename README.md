Robot Controller Service
==================

Fuse the robot pose measured by all the available cameras and use it to compute the velocity commands that need to be sent to the robot in order to execute a given task.

Dependencies:
-----
This service depends on the following services:

* [is-broker-events](https://github.com/labviros/is-broker-events): Used to check which cameras are available.
* [is-frame-transformation](https://github.com/labviros/is-frame-transformation): Used grab the transformations from the robot frame to the world frame using the available cameras. Note that the transformations should be available somehow, this may imply a dependency on other services like [is-aruco-detector](https://github.com/labviros/is-aruco-detector), for instance.
* [is-robots](https://github.com/labviros/is-robots): This service expects the robot to have a gateway compliant with the standard API. Particularly, it expects the robot to have **RobotGateway.{robot_id}.SetConfig** implemented.


RPCs
------
| Service | Request | Reply | Description | 
| ------- | ------- | ------| ----------- |
| RobotController.{robot_id}.SetTask | [RobotTask] | Empty | Configure the current task to be executed. |

[RobotTask]: https://github.com/labviros/is-msgs/tree/master/docs#is.robot.RobotTask