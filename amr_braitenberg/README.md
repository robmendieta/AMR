Summary
=======

Implements three types of Braitenberg vehicles. The original vehicles are
supposed to be two-wheeled differential drives. Since our platform is an
omni-directional youBot, the differential drive is simulated by the
`differential_drive_emulator` node. The logic of the Braitenberg vehicle itself
is implemented by the `braitenberg_vehicle` node.

The launch file "*braitenberg.launch*" brings up the Stage simulator with
"*Octopus*" world (which is well-suited for testing braitenberng vehicles),
both nodes from this package,  and dynamic reconfigure GUI.

![Octopus world](../amr_stage_worlds/bitmaps/octopus.png)

Nodes
=====

braitenberg\_vehicle
--------------------

This node supports dynamic reconfiguration during runtime. Vehicle type and the
connection factors could be adjusted through the `reconfigure_gui`.

### Subscribed topics

* `sonar_braitenberg` (*amr_msgs/Ranges*)
  range readings from the pair of braitenber sonars

### Published topics

* `cmd_vel_diff` (*amr_msgs/WheelSpeed*)
  array of two wheel speeds

differential\_drive\_emulator
-----------------------------

### Subscribed topics

* `cmd_vel_diff` (*amr_msgs/WheelSpeed*)
  array of two wheel speeds

### Published topics

* `cmd_vel` (*geometry_msgs/Twist*)
  velocity commands to control the robot

### Parameters

* `~wheel_diameter` (default: 0.15)
  wheel diameter in meters

* `~distance_between_wheels` (default: 0.5)
  distance between wheels in meters
