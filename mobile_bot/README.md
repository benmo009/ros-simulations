# Mobile Bot  

Package for mobile robot simulation using ROS.  

## TODO  

- Implement range sensors in room corners, and bearing sensor
- Implement kalman filter for range sensors and bearing sensor
- Implement mapping algorithm
- Implement localization with laser scanners as range sensor
- Implement amcl

## Kalman Filter for Dead Reckoning  

Subscribe to differential drive odometry topic --> gets estimation of command input  
Get initial robot state w/ noise  
Initialize Kalman Filter (Functions and everything)  
Every 1 second, use Kalman filter to estimate new state  
