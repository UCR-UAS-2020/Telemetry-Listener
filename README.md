# Telemetry-Listener

*** telemetry_node ***
Contains a ROS node that can subscribe to the global_position/global topic published by an active mavros node collecting GPS data and forward
the data over a serial port. Also contains an Arduino sketch that allows an Arduino to recieve the data over serial and compute the angles
needed to point the antenna tracker (this requires the Arduino to detect its own GPS coordinates).
*** servo_control_node (UNUSED) ***
Template for a ROS node used to control servos by sending commands to a maestro servo controller.
*** telemetry_listener (OUTDATED) ***
Contains a ROS node that can subscribe to the global_position/global topic published by an active mavros node actively collecting GPS data. The node also listens to the antennaTrackerGPS topic for the GPS coordinates of the tracker itself. The GPS coordinates are converted into euler angles which are published on the targetAngle topic. The telemetry_listener(source) file contains the source code for this package.
*** custom_apm.launch ***
The file called custom_apm.launch is a launch file for mavros, for more on how to use it see the doc titled "How to setup bridge from Navio2 to QGC using Mavros as the intermediary" in the project drive.