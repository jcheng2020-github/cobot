The packages for hardware interfaces, hardware drivers, control, robot model description, microcontroller scripts, moveit configure files, and deviation correction plugin.
# Software famework
We use Robotic Operate System (ROS Noetic Ninjemys), an open-source robotics middleware suite, for robot software development. A set of packages that include controller interfaces, controller managers, transmissions, and hardware interfaces called ros_control are applied. The ros_control packages take as input the joint state data from the robot's actuator's encoders and an input set point. It uses a generic control loop feedback mechanism, a PID controller, to control the joint position outputs.
## ar3_hardware_interface, ar3_hardware_driver, gripper_hardware_interface and gripper_hardware_driver
The ar3 and gripper hardware Interfaces are used by ros_ control in conjunction with position_controllers with the help of ar3 and gripper drivers to send and receive commands to the Arduino Mega 2560, Teensy 3.5 or Teensy 4.1 by Boost.Asio which is a C++ library with a consistent asynchronous I/O model for commanding position-based joints.
![](ros_control.png)

(Credit: http://wiki.ros.org/ros_control)

The string type variable is as the media of the asynchronous I/O. The header of each string represents different purpose of the commands:
| Sent to Teensy 4.1  | Response from Teensy 4.1 |
| ------------- | ------------- |
| MT: update latest position commands  | JP: update encoder steps  |
| JP: get joint position  | JP: update encoder steps  |
| JC: calibrate joints  | JC: update encoder calibration data  |


## MoveIt and Rviz
The motion planning framework called MoveIt has access to the ros_control nodes, the ar3 and gripper hardware interfaces, through a plugin called MoveItControllerManager. For trajectory calculation, the KDL kinematics plugin of MoveIt wraps around the numerical inverse kinematics solver provided by the Orocos KDL package are used, which only works with serial chains. Rviz is the primary visualizer in ROS. The MoveIt Rviz plugin allows us to set up virtual environments, create start and goal states for the robot interactively, test various motion planners, and visualize the output.


Topic `/ar3_controller/state` and `/gripper_controller/state` all track cobot state in their corresponding joints. `ar3_hardware_interface` and `gripper_hardware_interface` publish these topic separately. As long as `/ar3_controller` and `/gripper_controller` are assigned to `controller_spawner` node separately in their launch files. It is required for controller name to match the topic name requirements given by MoveIt package ( especially, a series of `/follow_joint_trajectory` topic such as:<br />
`/ar3_controller/follow_joint_trajectory/cancel`<br />
`/ar3_controller/follow_joint_trajectory/feedback`<br />
`/ar3_controller/follow_joint_trajectory/goal`<br />
`/ar3_controller/follow_joint_trajectory/result`<br />
`/ar3_controller/follow_joint_trajectory/status`<br />
`/gripper_controller/follow_joint_trajectory/cancel`<br />
`/gripper_controller/follow_joint_trajectory/feedback`<br />
`/gripper_controller/follow_joint_trajectory/goal`<br />
`/gripper_controller/follow_joint_trajectory/result`<br />
`/gripper_controller/follow_joint_trajectory/status`)<br />
The `/ar3_controller/controllers/state` and `/gripper_controller/controllers/state` are all assigned to the `controller_spawner` node in order to add cobot state tracking information to `/joint_states` topic and make sure `/joint_states` includes joints data from both ar3 and gripper. Hence, `/ar3_controller/state`, `/gripper_controller/state` and `/joint_states` all include cobot pose tracking information. And `/joint_states` has two publisher nodes: `/gripper_hardware_interface` and `/ar3_hardware_interface`. The subscriber node `move_group` only listens to `/joint_states`.<br />
WARN: if response time from hardware is too long, then, MoveIt trajectory control will have error of the control target such as submit a commend which is out of range of joints motion.


## MoveIt ERROR:
The plugin for class ‘moveit_rviz_plugin/MotionPlanning” failed to load.<br />
Solution:<br />
sudo apt update<br />
sudo apt upgrade<br />
sudo apt install ros-noetic-moveit-simple-controller-manager<br />
sudo apt update<br />
sudo apt upgrade
