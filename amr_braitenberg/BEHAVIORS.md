# Braitenberg vehicle behavior description
# author Roberto Mendieta
--------
1. Configuration A: 
* Right sonar value goes to right wheel speed
* Left sonar value goes to left wheel speed
* Robot goes along x axis, if it senses an obstacle turns towards the obstacle

2. Configuration B: 
* Right sonar value goes to left wheel speed
* Left sonar value goes to right wheel speed
* Robot goes straight, avoids wall obstacles and follows octopus.world path

3. Configuration C: 
* Right and left sonar values are added together and sent to each wheel
* Each wheel speed is then multiplied by its corresponding factor 
* Robot goes straight, changing factor to negative changes direction
