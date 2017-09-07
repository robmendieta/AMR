# AMR
Autonomous Mobile Robots assignments repository

All manuals and other information can be found in the wiki at https://mas.b-it-center.de/gitgate/amr-lab/amr-public/wikis/home

## Assignment description

### Braitenberg vehicle implementation.

- Robot with reactive behavior.

- Package: amr_braitenberg

- File: src/braitenberg_vehicle.py

### Motion controller.

- Velocity control for an omnidirectional robot's linear and angular speeds.

- Package: amr_navigation

- File: nodes/motion_controller.py

### Wallfollower

- SMACH implementation of a wallfollowing robot.

- Package: amr_bugs

- File: src/amr_bugs/wallfollower_state_machine.py


### "Bug2" algorithm for obstacle avoidance

- Package: amr_bugs

- File: src/amr_bugs/bug_brain.py


### Path executor

- Move to a series of poses, one at a time. Additionally, determine if any pose would be unreachable and allow preemption.

- Package: amr_navigation

- File: nodes/path_executor.py

### Sonar mapper 

- Probabilistic egocentric robot mapping 

- Package: amr_mapping

- File: src/amr_mapping/sonar_map.py

### Pose likelihood

- Estimate a robot's pose given laser sensor readings, used for particle filter.

- Package: amr_localization

- File: nodes/pose_likelihood_server.py

### Particle filter for robot localization

- Package: amr_localization

- File: src/amr_localization/motion_model.py, src/amr_localization/particle_filter.py

### Path planner

- “Randomized Roadmap Planner” algorithm.

- Package: amr_navigation

- File: src/amr_navigation/randomized_roadmap_planner.py