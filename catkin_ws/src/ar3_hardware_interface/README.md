# ar3_hardware_interface
This module contains the hardware interface for the AR3. 

## Position Controller
The hardware_interface implements a Position Joint Trajectory Controller from the [ros_control](http://wiki.ros.org/ros_control) framework. It serves as an interface between the higher level planning (MoveIt) and the actuator driver (Teensy driver).

## Joint Encoder Calibration
You can skip the automatic encoder calibration sequence with the parameter `use_existing_calibrations:=true`. Since the original encoders (AMT-102V) are relative encoders, they need to be calibrated against the limit switches when powered on. If you are restarting the software but not the Teensy, the encoders will have remained powered on and do not need to be recalibrated. Alternatively, you can change the REST_ENC_POSITIONS values in the Teensy sketch so that the encoders are initialised to those values on startup, provided that your arm is also always initialised in that position as well. Before starting any movements, it is always good to check in RViz that the model is in a sensible position to verify that the encoders are properly calibrated.
