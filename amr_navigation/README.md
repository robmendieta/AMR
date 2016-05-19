Summary
=======

Navigation package.

Nodes
=====

motion\_controller
------------------

This node is a motion controller which allows to drive the robot to a certain
position in the global coordinate frame.

The motion controller has two modes of operation (which we call velocity
controllers):

* *DiffVelocityController*, which decomposes the required movement into angular
  and linear components, and executes them separately. Moreover, it drives the
  robot at a constant (max) velocity until it has almost reached the goal pose,
  then it switches to the minimum velocity.

* *OmniVelocityController*, which executes both angular and linear motion at
  the same time, and slows down smoothly as it approaches the target pose.

In order to increase the safety of the robot, the node may listen to the
`obstacles` topic and immediately abort the current active goal upon reception
of a message in this topic. This capability could be switched with
`~abort_if_obstacle_detected` parameter.

### Action API

The `motion_controller` node provides an implementation of the
*SimpleActionServer* (see [actionlib][] documentation), that takes in goals
containing *geometry_msgs/PoseStamped* messages. The user can communicate with
the `motion_controller` node over ROS topics directly, but the recommended way
to send goals is by using the *SimpleActionClient*.

### Subscribed topics

* `~move_to_simple/goal` (*geometry_msgs/PoseStamped*)  
  provides a non-action interface for users that do not care about tracking the
  execution status of their goals

* `obstacles` (*amr_msgs/Obstacle*)  
  notifications of obstacles detected on the way of the robot

### Published topics

* `cmd_vel` (*geometry_msgs/Twist*)  
  velocity commands to control the robot

* `~current_goal` (*geometry_msgs/PoseStamped*)  
  the goal that the controller is currently trying to achieve

### Parameters

* `~controller_frequency` (default: 10.0)  
  how often the velocity command is updated and published

* `~abort_if_obstacle_detected` (default: true)  
  whether to abort the current action if an obstacle was detected

* `~max_linear_velocity` (default: 0.3)  
  maximum allowed linear velocity

* `~max_linear_acceleration` (default: 0.05)  
  maximum allowed linear acceleration

* `~linear_tolerance` (default: 0.02)  
  maximum allowed distance to the target position at which it is considered to
  be reached

* `~max_angular_velocity` (default: 0.2)  
  maximum allowed angular velocity

* `~max_angular_acceleration` (default: 0.03)  
  maximum allowed angular acceleration

* `~angular_tolerance` (default: 0.02)  
  maximum allowed angular distance to the target orientation at which it is
  considered to be reached

* `~controller` (default: "omni")  
  velocity controller to use

path\_executor.py
-----------------

This node allows to drive the robot through a path (list of poses). Depending on
the settings the motion between the poses may involve local obstacle avoidance.

### Action API

The `path_executor` node provides an implementation of the
*SimpleActionServer* (see [actionlib][] documentation), that takes in goals
containing *nav_msgs/Path* messages. The user can communicate with it by using
the *SimpleActionClient*.

### Subscribed actions

* `/motion_controller/move_to` (*amr_msgs/MoveToAction*)  
  action to move the robot to a specific pose (no obstacle avoidance)

* `/bug2/move_to` (*amr_msgs/MoveToAction*)  
  action to move the robot to a specific pose (with Bug2 obstacle avoidance)

### Parameters

* `~use_obstacle_avoidance` (default: false)  
  controls which provider of *[move_to]* action is used

[actionlib]: http://www.ros.org/wiki/actionlib
