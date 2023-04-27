//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>
#include "chess_player/chess_evaluator.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

const int COLUMN_NUMBER = 8;
const int ROW_NUMBER = 8;
bool input[ROW_NUMBER][COLUMN_NUMBER];
box detail;
std::string result;
bool broadcast = false;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	char display[49];
	static long long iteration = 0;
	static bool historyInput[ROW_NUMBER][COLUMN_NUMBER]= 
  {{false, false, false, false, false, false, false, false},//1 row
   {false, false, false, false, false, false, false, false},//2 row
   {false, false, false, false, false, false, false, false},//3 row
   {false, false, false, false, false, false, false, false},//4 row
   {false, false, false, false, false, false, false, false},//5 row
   {false, false, false, false, false, false, false, false},//6 row
   {false, false, false, false, false, false, false, false},//7 row
   {false, false, false, false, false, false, false, false}};//8 row
//  h	    g	   f	  e	  d      c      b	a	column
	//recode the reading form sensor chessboard in input array
	
	for( int i = 0; i < 49 ; i ++)
	{
		display[i] = msg->data.c_str()[i];
	}
	if(display[0] != 'i')
	{
		for( int i = 0; i < COLUMN_NUMBER; i++)
		{
			if(display[1 + i * 2] == '1')
			{
				input[display[0] - '1'][i] = true;
			}
			else
			{
				input[display[0] - '1'][i] = false;
			}
		}
	}
	else
	{
		if(display[1] == 'S')
		{
			broadcast = true;
		}
		else
		{
			broadcast = false;
		}
	}
	if(iteration >= 9 )
	{
	//evaluate
		for(int i = 0; i < ROW_NUMBER; i ++)
		{
			for(int j = 0; j < COLUMN_NUMBER; j ++)
			{
				if(historyInput[i][j] != input[i][j])
				{
					if(historyInput[i][j] == false)
					{
						detail.update(i, j, become_occupied);
					}
					else
					{
						detail.update(i, j, become_empty);
					}
					result = detail.evaluate();
				}
			} 
		}
	}	
	for(int i = 0; i < ROW_NUMBER ; i++)
	{
		for(int j = 0; j < COLUMN_NUMBER; j++)
		{
			historyInput[i][j] = input[i][j];
		}
	}
	
	if(iteration / 9 % 10 == 0 && iteration % 9 == 0)
	{
		std::cout << "iteration number: " << iteration << std::endl;
		char rowLabel = '8';
		for(int i = 0 ; i < ROW_NUMBER ; i++, rowLabel--)
		{
			std::cout << rowLabel << ": ";
			for(int j = 0; j < COLUMN_NUMBER; j++)
			{
				if( input[ROW_NUMBER - 1 - i][COLUMN_NUMBER - 1 - j] == true)
				{
					std::cout << "|1";  
				}
				else
				{
					std::cout << "| ";
				}
			}
			std::cout << "|" << std::endl;
		}
		std::cout << "    a b c d e f g h " << std::endl;
		detail.displayLog();
	}
	
	
	if(iteration / 9 % 10 == 9)
	{
		system("clear");
	}
	iteration++;
}

int main(int argc, char **argv)
{
	system("clear");
	ros::init(argc, argv, "chess_move_monitor");
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/interpretation_of_sensor_chessboard", 1000);
	ros::Rate loop_rate(10);
	
	ros::Subscriber sub = n.subscribe("msgs_from_sensor_chessboard", 1000, chatterCallback);
	
	while (ros::ok())
	{
		if(broadcast == true)
		{
			std_msgs::String msg;
		
			msg.data = result.c_str();
			std::cout << "broadcasting evaluation result: " << msg.data << std::endl;
			chatter_pub.publish(msg);
			broadcast = false;
		}
		
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
