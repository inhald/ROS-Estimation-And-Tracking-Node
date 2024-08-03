### Vehicle Tracking Node
A ROS Noetic compatible vehicle tracking node for position and orientation in 3-d space. The node subscribes to /tag_detections, which publishes the frame of reference of an AprilTag. Using this data, a Kalman Filter is applied at every callback, which facilitates the tracking. 

## External Dependencies
+ apriltag-ros package [^1]: this must be installed and the tag on the tracked vehicle must be configured. Further information can be found on the apriltag-ros github.


This package is maintained by Neelkant Teeluckdharry [teeluckn@mcmaster.ca].


[^1]: https://github.com/AprilRobotics/apriltag_ros
