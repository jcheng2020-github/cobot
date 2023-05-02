The workspace for ar3 and gripper control
## Activate the arm:
* Check the connection for the arm in the main box and release the emergency read button.
* Connect Teensy 4.1 to port /dev/ttyACM0, Arduino Mega 2560 (gripper) to port /dev/ttyACM1 and Arduino Mega 2560 (sensor chessboard) to port /dev/ttyACM2 separately. The operating system may assign ports names differently, the Arduino IDE tools bars can help users check the actual port names and users are required to change the corresponding names in configure files in packages to guarantee the PC can build communications with Teensy 4.1 and two Arduino Mega 2560.
* Open Arduino IDE and burn baseline_with_ARCS_jcheng2020_revised_asyn_IO_com_version script into Teensy 4.1 (https://github.com/jcheng2020-github/cobot/tree/main/catkin_ros/src/cobot_microcontrollers/teensy/baseline_with_ARCS_jcheng2020_revised_asyn_IO_com_version),
* Burn gripper_driver_microcontroller into Arduino Mega 2560 (gripper) (https://github.com/jcheng2020-github/cobot/tree/main/catkin_ros/src/cobot_microcontrollers/arduino_mega/gripper_driver_microcontroller)
* Burn sensor_chessboard_script int Arduino Mega 2560 (sensor chessboard) (https://github.com/jcheng2020-github/cobot/tree/main/catkin_ros_chessboard/src/chessboard_microcontrollers/sensor_chessboard_script)
* Set up gripper circuit following instruction in https://github.com/jcheng2020-github/cobot/tree/main/catkin_ros/src/cobot_microcontrollers/arduino_mega/gripper_driver_microcontroller
* Turn on the 7.5 volts DC power supply, then check whether the gripper performs an open and close motion group. If the gripper performs open and close motions fluently, the circuit is good, else rewriting the circuit is required.
* Open a new terminal window and change the directory path to catkin_ros workspace.
* Add additional four terminals in the same window (Optional but recommended).
* Check whether all these terminals are in the catkin_ros workspace directory.
* Run 
```
source ./devel/setup.bash
```
at each terminal
* Check the connection of all limit switches on the arm with the power supply and Teensy 4.1 pins, any issues with the connection of the limit switches may cause irreversible damage to the arm.
* Run
```
roslaunch ar3_hardware_interface ar3_hardware_bringup.launch
```
on the first terminal<br />
If the terminal shows that **"Failed to connect to serial port /dev/ttyACM0"**, it means that the USB port connected to Teensy 4.1 is not connected well.<br /> 
When the terminal shows **"Running joint calibration…"**, the 6 joints of the AR3 arm should begin their rotation one by one to finish their calibration.<br />
If the arm has already been calibrated following ar3_hardware_interface commands before, users can choose to run “roslaunch ar3_hardware_interface ar3_hardware_bringup.launch use_existing_calibrations:=true” instead.<br />
If any anomaly happens during the calibration procedure, input ctrl C on the ar3_hardware_interface terminal, then burn the `baseline_with_ARCS_jcheng2020_revised_asyn_IO_com_version` Arduino script into Teensy 4.1 again or reset the Teensy. Next, run
```
roslaunch ar3_hardware_interface ar3_hardware_bringup.launch
```
in ar3_hardware_interface terminal.<br />
* Run
```
roslaunch gripper_hardware_interface gripper_hardware_bringup.launch
```
on the second terminal for gripper_hardware_interface.<br />
If gripper_hardware_interface terminal shows **"Failed to start controllers: /gripper_controller/controllers/state, /gripper_controller"**, input ctrl C on the gripper_hardware_interface terminal and run “roslaunch gripper_hardware_interface gripper_hardware_bringup.launch” again.<br />
Run 
```
roslaunch cobot_moveit_config cobot_moveit_bringup.launch
```
on the third terminal for MoveIt. Then, the following GUI will be displayed:
![](MoveIt%202023-05-02%2011-12-47.png)

* Click `“Fixed Frame”` of `“Global Option”` of `“Display”` and select `“world”` as the fixed frame.
* Click the `“Add”` button at the left column and a new dialog box will appear as shown below:


