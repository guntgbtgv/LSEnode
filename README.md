LSEnode
=======

This ros-node represents a simple interface to the LSE library. It does by far not support the features of the library.
In short:
	- Initialize the LSE-manager (while loading the Parameters.xml parameter-file)
	- Subscribes to the measurement topics (can be specified in the launch file)
	- Resets the filter state on the first measurement
	- Passes the measurement data further to the filter
	- Updates the filter if an encoder measurement is available
	- Publishes the estimated state to ros-topics (the launch file launches corresponding rxplots)

INSTALLATION:
- add directory to ROS
- enter build folder
- execute cmake: "cmake .."
- compile: "make"

DEPENDENCIES:
- Standard library
- ROS (including sensor_msgs and geometry_msgs)
- Eigen3
- LSE library (if you did not use the install flag during compilation you will have to link the header files and the library by hand)
