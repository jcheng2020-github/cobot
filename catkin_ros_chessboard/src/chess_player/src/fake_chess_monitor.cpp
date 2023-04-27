#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_chess_monitor");
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("interpretation_of_sensor_chessboard"/*"interpretation_of_sensor_chessboard"*/, 1000);
	ros::Rate loop_rate(10);
	
	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;
		
		std::stringstream ss;
		std::string temp;
		std::cout << "Input the broadcasted msg: ";
		std::cin >> temp;
		ss << temp;
		msg.data = ss.str();
		
		chatter_pub.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
