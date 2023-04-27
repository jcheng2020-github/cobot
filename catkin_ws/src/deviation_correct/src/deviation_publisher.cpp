 //jcheng2020 devaition data publisher for joint manually adjustment
 #include "ros/ros.h"
 #include "std_msgs/String.h"
 #include <iostream>
 #include <sstream>

 
 int main(int argc, char **argv)
 {
	 ros::init(argc, argv, "talker");
	 ros::NodeHandle n;
	 ros::Publisher chatter_pub = n.advertise<std_msgs::String>("deviation_msgs", 1000);
	 
	 ros::Rate loop_rate(1);
	 
	 double joint_deviation[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
	 
	 char controlKey[6] = {'a','s','d','f','g', 'h'};
	 
	 while (ros::ok())
	 {
		 std_msgs::String msg;
		 std::string media;
		 std::stringstream stream;
		 std::string finalOutput = "";
		 char letter;
		 char inputLetter;
		 int targetIteration;
		 
		 
		 
		 char instruction;
		 
		 
		 
		 
		 
		 std::cout << "Please, enter your deviation ajustment parameters:" << std::endl;
		 letter = 'A';
		 std::cout << "Option:" << std::endl
		 << "joint_1: a A"<< std::endl
		 << "joint_2: s S"<< std::endl
		 << "joint_3: d D"<< std::endl
		 << "joint_4: f F"<< std::endl
		 << "joint_5: g G"<< std::endl
		 << "joint_6: h H"<< std::endl;
		 std::cout << "Input:   ";
		 std::getline (std::cin, media);
		 system("clear");
		 instruction = media[0];
		 stream.str(std::string());
		 bool check = false;
		 for(int i = 0; i < 6; i ++)
		 {
		 	if(instruction == controlKey[i] || instruction == controlKey[i] - 32)
		 	{
		 		inputLetter = 'A' + i;
		 		if(instruction == controlKey[i])
			 	{
			 		stream << std::fixed << std::setprecision(3) << joint_deviation[i] - 0.2;
			 		media = stream.str();
			 	}
			 	if(instruction == controlKey[i] - 32)
			 	{
			 		stream << std::fixed << std::setprecision(3) << joint_deviation[i] + 0.2;
			 		media = stream.str();
			 	}
			 	check = true;
		 	}
		 }
		 if(check == false)
		 {
		 	continue;
		 }
		 
		 targetIteration = inputLetter - 'A';
		 
		 
		 
		 
		 
		 /*
		 std::cout << "Please, enter your deviation ajustment parameters:" << std::endl;
		 letter = 'A';
		 std::cout << "Example: " << char(letter) <<"1.478" << std::endl;
		 std::cout << "Input:   ";
		 inputLetter = getchar();
		 targetIteration = inputLetter - 'A';
		 std::cout << targetIteration << std::endl;
		 std::getline (std::cin, media);
		 */
			 
		 for(int i = 0, letter = 'A'; i < 6; i++, letter++)
		 {
		 	std::string output;
		 	stream.str(std::string());
			if(i != targetIteration)
			{
				std::stringstream streamTemp;
				streamTemp << std::fixed << std::setprecision(3) << joint_deviation[i];
				output = streamTemp.str();
			}
			else
			{
				output = media;
				joint_deviation[i] = std::stod(output);
			}

			 stream << std::fixed << std::setprecision(3) << std::stod(output);
			 finalOutput.push_back(letter);
			 finalOutput += stream.str();
		 }
		 msg.data = finalOutput;
		 ROS_INFO("%s", msg.data.c_str());
		 chatter_pub.publish(msg);
		 ros::spinOnce();
		 loop_rate.sleep();
	 }
	 return 0;
 }
