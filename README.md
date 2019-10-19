# Telemetry-Listener
Contains a ROS node that can subscribe to the global_position/global topic published by an active mavros node actively collecting GPS data.
The telemetry_listener(source) file contains the source code for this package. The file called telemetry_listener is the package built using catkin_make install.
The file called custom_apm.launch is a launch file for mavros, for more on how to use it see the doc titled "How to setup bridge from Navio2 to QGC using Mavros as the intermediary" in the project drive.