//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>

#include <ros/callback_queue.h>
#include <gripper_hardware_interface/gripper_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_hardware_interface");
	ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
  gripper_hardware_interface::GRIPPERHardwareInterface gripper(nh);

	ros::MultiThreadedSpinner spinner(0);
	spinner.spin(&ros_queue);

  return 0;
}
