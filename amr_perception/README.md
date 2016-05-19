Summary
=======

Nodes
=====

obstacle\_detector
------------------

This node listens to velocity commands and pioneer sonar readings. Based on the
former it decides what is the current motion direction. It then considers the
subset of the sonars that face in the current motion direction and make sure
that their range readings are larger that the safe distance threshold. If not,
it sends a notification of the detected obstacle to a special topic.

This node supports dynamic reconfiguration during runtime. The safe distance
threshold could be adjusted through the `reconfigure_gui`.

### Subscribed topics

* `cmd_vel` (*geometry_msgs/Twist*)  
  velocity commands to control the robot

* `sonar_pioneer` (*amr_msgs/Ranges*)  
  range readings from the pioneer sonar ring

### Published topics

* `obstacles` (*amr_msgs/Obstacle*)  
  notifications of detected obstacles
