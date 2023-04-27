//jcheng2020 revised version for joint manually adjustment   
//(asynchronous IO model include boost communicate version)
#include <ros/callback_queue.h>
#include <ar3_hardware_interface/ar3_hardware_interface.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
 {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
 }
 
//joint_deviation_adjustment define
double ar3_hardware_interface::AR3HardwareInterface::joint_deviation_adjustment[6];



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar3_hardware_interface");
	ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
  ar3_hardware_interface::AR3HardwareInterface ar3(nh);

  



	ros::MultiThreadedSpinner spinner(0);
	spinner.spin(&ros_queue);

  return 0;
}
