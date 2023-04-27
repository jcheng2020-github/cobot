//<description>MoveIt script for chess_player_motion_series test MOVEIT P&P SOLUTION TEST move_group_demo( with grasp msgs)2 </description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>
//<author email="jcheng2020@my.fit.edu">Junfu Cheng</author>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>





#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>









#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <string>




const int COLUMN_NUMBER = 8;
const int ROW_NUMBER = 8;
bool input[ROW_NUMBER][COLUMN_NUMBER];

std::string result;
bool broadcast = false;







enum GripperInstruction{Open, Close};



// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


//object postion parameters
//hortizontal with no rotation range object_x: [-0.4 , 0.4] object_y: [-0.55, -0.65] division value: 0.003
//vetical range object_x, end_x: [-0.2 , 0.2] object_y, end_y: [ -0.50 , -0.18] division value: 0.003
double object_x = -0.20, object_y = -0.00 , object_z = 0.110;// object_z = 0.035




const double tfXOrigin_begin = 0.157;
	//dX1_begin = 0.052 , 
	//dX2_begin = 0.052
const double tfYOrigin_begin = -0.201;
	//dY1_begin = 0.044
	//dY2_begin = 0.038
	
	
const double piecePositionZ = 0.1045 /*0.100 + 0.003*/;



//double tfX_begin[9] = {tfXOrigin_begin ,
//	         tfXOrigin_begin - dX1_begin/*0.115*/,
//		 tfXOrigin_begin - dX1_begin *2 , 
//		 tfXOrigin_begin - dX1_begin *3, 
//		 tfXOrigin_begin - dX1_begin *4,
//		 tfXOrigin_begin - dX1_begin *5,
//		 tfXOrigin_begin - dX1_begin *6,
//		 tfXOrigin_begin - dX1_begin *7,
//		 tfXOrigin_begin - dX1_begin *8};
double tfX_begin[9] = {tfXOrigin_begin ,
 tfXOrigin_begin - 0.047,
 tfXOrigin_begin - 0.047 - 0.043,
 tfXOrigin_begin - 0.047 - 0.043 - 0.046,
 tfXOrigin_begin - 0.047 - 0.043 - 0.046 - 0.047,
 tfXOrigin_begin - 0.047 - 0.043 - 0.046 -0.047 -0.046,
 tfXOrigin_begin - 0.047 - 0.043 - 0.046 -0.047 -0.046 - 0.046,
 tfXOrigin_begin - 0.047 - 0.043 - 0.046 -0.047 -0.046 - 0.046 -0.046,
 tfXOrigin_begin - 0.047 - 0.043 - 0.046 -0.047 -0.046 - 0.046 -0.046 - 0.056};
//double tfY_begin[8] = {tfYOrigin_begin ,
//		 tfYOrigin_begin - dY1_begin/*-0.253*/,
//		 tfYOrigin_begin - dY1_begin *2,
//		 tfYOrigin_begin - dY1_begin *3, 
//		 tfYOrigin_begin - dY1_begin *4, 
//		 tfYOrigin_begin - dY1_begin *5,
//		 tfYOrigin_begin - dY1_begin *6,
//		 tfYOrigin_begin - dY1_begin *7};
double tfY_begin[8] = {tfYOrigin_begin,
tfYOrigin_begin - 0.044,
tfYOrigin_begin - 0.044 - 0.044,
tfYOrigin_begin - 0.044 - 0.044 - 0.044,
tfYOrigin_begin - 0.044 - 0.044 - 0.044 - 0.038,
tfYOrigin_begin - 0.044 - 0.044 - 0.044 - 0.038 - 0.038,
tfYOrigin_begin - 0.044 - 0.044 - 0.044 - 0.038 - 0.038 - 0.047,
tfYOrigin_begin - 0.044 - 0.044 - 0.044 - 0.038 - 0.038 - 0.047 - 0.041};
		 
std::string implementation = "none";



void addCollisionChessboard(std::vector<moveit_msgs::CollisionObject>& collision_objects, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
	bool whetherRepeat = false;
	int object_number = collision_objects.size();
	
	for(int i= 0; i < object_number; i++)
	{
		if(collision_objects[i].id == "chessboard")
		{
			whetherRepeat = true;
		}
	}
	if(whetherRepeat == false)
	{
		collision_objects.resize(object_number + 1);
		collision_objects[object_number].id = "chessboard";
		collision_objects[object_number].header.frame_id = "base_link";
		
		/* Define the primitive and its dimensions. */
		collision_objects[object_number].primitives.resize(1);
		
		collision_objects[object_number].primitives[0].type = collision_objects[object_number].primitives[0].BOX;
		collision_objects[object_number].primitives[0].dimensions.resize(3);
		collision_objects[object_number].primitives[0].dimensions[0] = 0.440;
		collision_objects[object_number].primitives[0].dimensions[1] = 0.380;
		collision_objects[object_number].primitives[0].dimensions[2] = 0.056;
		
		
		
		/* Define the pose of the piece. */
		collision_objects[object_number].primitive_poses.resize(1);
		collision_objects[object_number].primitive_poses[0].position.x = (tfX_begin[0] + tfX_begin[7]) / 2;
		collision_objects[object_number].primitive_poses[0].position.y = (tfY_begin[0] + tfY_begin[7]) / 2;
		collision_objects[object_number].primitive_poses[0].position.z = 0.028;
		collision_objects[object_number].primitive_poses[0].orientation.w = 1.0;
		collision_objects[object_number].operation = collision_objects[object_number].ADD;
		planning_scene_interface.addCollisionObjects(collision_objects);
		
		//add table
		object_number ++;
		collision_objects.resize(object_number + 1);
		collision_objects[object_number].id = "table";
		collision_objects[object_number].header.frame_id = "base_link";
		
		/* Define the primitive and its dimensions. */
		collision_objects[object_number].primitives.resize(1);
		
		collision_objects[object_number].primitives[0].type = collision_objects[object_number].primitives[0].BOX;
		collision_objects[object_number].primitives[0].dimensions.resize(3);
		collision_objects[object_number].primitives[0].dimensions[0] = 5;
		collision_objects[object_number].primitives[0].dimensions[1] = 5;
		collision_objects[object_number].primitives[0].dimensions[2] = 0.056;
		
		
		
		/* Define the pose of the piece. */
		collision_objects[object_number].primitive_poses.resize(1);
		collision_objects[object_number].primitive_poses[0].position.x = 0;
		collision_objects[object_number].primitive_poses[0].position.y = 0;
		collision_objects[object_number].primitive_poses[0].position.z = -0.028;
		collision_objects[object_number].primitive_poses[0].orientation.w = 1.0;
		collision_objects[object_number].operation = collision_objects[object_number].ADD;
		planning_scene_interface.addCollisionObjects(collision_objects);
		
		//add driver box
		object_number ++;
		collision_objects.resize(object_number + 1);
		collision_objects[object_number].id = "driverBox";
		collision_objects[object_number].header.frame_id = "base_link";
		
		/* Define the primitive and its dimensions. */
		collision_objects[object_number].primitives.resize(1);
		
		collision_objects[object_number].primitives[0].type = collision_objects[object_number].primitives[0].BOX;
		collision_objects[object_number].primitives[0].dimensions.resize(3);
		collision_objects[object_number].primitives[0].dimensions[0] = 0.390;
		collision_objects[object_number].primitives[0].dimensions[1] = 0.490;
		collision_objects[object_number].primitives[0].dimensions[2] = 0.200;
		
		
		
		/* Define the pose of the piece. */
		collision_objects[object_number].primitive_poses.resize(1);
		collision_objects[object_number].primitive_poses[0].position.x = 00.450;
		collision_objects[object_number].primitive_poses[0].position.y = 0;
		collision_objects[object_number].primitive_poses[0].position.z = 0.100;
		collision_objects[object_number].primitive_poses[0].orientation.w = 1.0;
		collision_objects[object_number].operation = collision_objects[object_number].ADD;
		planning_scene_interface.addCollisionObjects(collision_objects);
	}
}


void addCollisionPiece(std::string name, double x, double y, double z, std::vector<moveit_msgs::CollisionObject>& collision_objects, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
	const double pieceDiameter = 0.032;
	bool whetherRepeat = false;
	int object_number = collision_objects.size();
	
	for(int i= 0; i < object_number; i++)
	{
		if(collision_objects[i].id == name + "Head" ||collision_objects[i].id == name + "Body" )
		{
			whetherRepeat = true;
		}
	}
	
	if(whetherRepeat == false)
	{
		shape_msgs::SolidPrimitive primitive;
		collision_objects.resize(object_number + 1);
		collision_objects[object_number].id = name + "Body";
		  collision_objects[object_number].header.frame_id = "base_link";

		  /* Define the primitive and its dimensions. */
		  collision_objects[object_number].primitives.resize(1);
		  collision_objects[object_number].primitives[0].type = collision_objects[object_number].primitives[0].CYLINDER;
		collision_objects[object_number].primitives[0].dimensions.resize(2);;
		collision_objects[object_number].primitives[0].dimensions[primitive.CYLINDER_HEIGHT] = 0.075;
		collision_objects[object_number].primitives[0].dimensions[primitive.CYLINDER_RADIUS] = pieceDiameter / 2;

		  /* Define the pose of the piece. */
		  collision_objects[object_number].primitive_poses.resize(1);
		  collision_objects[object_number].primitive_poses[0].position.x = x;
		  collision_objects[object_number].primitive_poses[0].position.y = y;
		  collision_objects[object_number].primitive_poses[0].position.z = z - 0.01;
		  collision_objects[object_number].primitive_poses[0].orientation.w = 1.0;

		  collision_objects[object_number].operation = collision_objects[object_number].ADD;
		  planning_scene_interface.addCollisionObjects(collision_objects);
		  
		  object_number ++;
		  collision_objects.resize(object_number + 1);
		  collision_objects[object_number].id = name + "Head";
		  collision_objects[object_number].header.frame_id = "base_link";
		  collision_objects[object_number].primitives.resize(1);
		  collision_objects[object_number].primitives[0].type = collision_objects[object_number].primitives[0].BOX;
		  collision_objects[object_number].primitives[0].dimensions.resize(3);
		  collision_objects[object_number].primitives[0].dimensions[0] = 0.020;
		  collision_objects[object_number].primitives[0].dimensions[1] = 0.020;
		  collision_objects[object_number].primitives[0].dimensions[2] = 0.020;
		  /* Define the pose of the piece. */
		  collision_objects[object_number].primitive_poses.resize(1);
		  collision_objects[object_number].primitive_poses[0].position.x = x;
		  collision_objects[object_number].primitive_poses[0].position.y = y;
		  collision_objects[object_number].primitive_poses[0].position.z = z + 0.0375;
		  collision_objects[object_number].primitive_poses[0].orientation.w = 1.0;

		  collision_objects[object_number].operation = collision_objects[object_number].ADD;
		  planning_scene_interface.addCollisionObjects(collision_objects);
	  }
}



bool aim(double x, double y, double z, GripperInstruction clawInstruction, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
	moveit::planning_interface::MoveGroupInterface group("ar3");
	group.setPlanningTime(4.0);
	
	
	//Set maximum speed
    	group.setMaxVelocityScalingFactor(0.5);

    	//Set maximum acceleration
	group.setMaxAccelerationScalingFactor(0.5);
	
	  tf2::Quaternion orientation;
	  orientation.setRPY(tau / 2, 0, -tau / 2);//vertical pick
	  geometry_msgs::Pose target_pose1;
	  target_pose1.orientation = tf2::toMsg(orientation);
	  target_pose1.position.x = x;
	  target_pose1.position.y = y;
	  target_pose1.position.z = z;
	  group.setPoseTarget(target_pose1);
	  group.move();
	
	moveit::planning_interface::MoveGroupInterface group2("gripper");
	group2.setPlanningTime(1.0);
	const robot_state::JointModelGroup* joint_model_group =
	group2.getCurrentState()->getJointModelGroup("gripper");
	moveit::core::RobotStatePtr current_state = group2.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	if(clawInstruction == Open)
	{
		joint_group_positions[0] = 0.04;
		joint_group_positions[1] = 0.04;
	}
	if(clawInstruction == Close)
	{
		joint_group_positions[0] = 0.013;
		joint_group_positions[1] = 0.013;
	}
	group2.setJointValueTarget(joint_group_positions);
	group2.move();
	//check whether trajectory is feasible
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	return (group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
}

void aimWithCheck(double x, double y, double z, GripperInstruction clawInstruction, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
	double riseDistanceForPieceHeight = 0.110;
	double minRiseDistance1 = 0.015;
	double minRiseDistance2 = 0.001;
	moveit::planning_interface::MoveGroupInterface group("ar3");
	group.setPlanningTime(5.0);
	bool success = false;
	success = aim(x, y, z  + riseDistanceForPieceHeight, clawInstruction, planning_scene_interface);//pursue retrieve distance 0.110
	
	
	if(success == false)
	{
		ROS_INFO("backup aim");
		success = aim(x, y, z  + minRiseDistance1, clawInstruction, planning_scene_interface);
		if(success == false)
		{
			ROS_INFO("backup aim2");
			aim(x, y, z  + minRiseDistance2, clawInstruction, planning_scene_interface);
		}
	}
	else
	{
		ROS_INFO("Success aim");
	}
}

void addAllPieceOnBoard(std::string targetPosition, std::vector<moveit_msgs::CollisionObject>& collision_objects, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
	char row = targetPosition[0];
	char column = targetPosition[1];
	std::cout << std::endl<< targetPosition[0] << targetPosition[1] << std::endl;
	size_t rowIndex = ROW_NUMBER - 1 - (row - 'a');
	size_t columnIndex = column - '1';
	
	for(int i = 0; i < ROW_NUMBER; i++)
	{
		for(int j = 0; j < COLUMN_NUMBER; j ++)
		{
			std::string idSuffix = "";
			char pieceRowLabel('1' + i);
			char pieceColumnLabel('h' - j);
			idSuffix += pieceColumnLabel;
			idSuffix += pieceRowLabel;
			
			if( input[i][j] == true && (idSuffix != targetPosition))
			{
				std::cout << "Successful Add" << "piece " + idSuffix << std::endl;
				addCollisionPiece("piece" + idSuffix, tfX_begin[ pieceColumnLabel - 'a'],tfY_begin[ pieceRowLabel - '1'], piecePositionZ, collision_objects, planning_scene_interface);
			}
			else
			{
				std::cout << "Failed Add" << std::endl;
			}
		}
	}
	//planning_scene_interface.removeCollisionObjects("piece "+targetPosition);
}

void uci2tfPickPlace(std::string uci)
{
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "ar3_hand";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.003;
	ocm.absolute_y_axis_tolerance = 0.003;
	ocm.weight = 1.0;
	moveit_msgs::Constraints test_constraints;

	double object_x;
	double object_y;
	double object_z = 0.361 ;
	double end_x;
	double end_y;
	double end_z = 0.361 ;
	
	const size_t beginPosIndexUCI = 0;
	const size_t endPosIndexUCI = 2;
	const size_t posLengthUCI = 2;
	
	size_t beginTfPosReferenceX;
	size_t beginTfPosReferenceY;
	size_t endTfPosReferenceX;
	size_t endTfPosReferenceY;
	
	std::string beginPosUCI = uci.substr(beginPosIndexUCI,posLengthUCI);
	std::string endPosUCI = uci.substr(endPosIndexUCI,posLengthUCI);
	
	beginTfPosReferenceX = beginPosUCI[0] - 'a';
	beginTfPosReferenceY = beginPosUCI[1] - '1';
	
	endTfPosReferenceX = endPosUCI[0] - 'a';
	endTfPosReferenceY = endPosUCI[1] - '1';
	
	object_x = tfX_begin[beginTfPosReferenceX];
	object_y = tfY_begin[beginTfPosReferenceY];
	
	end_x = tfX_begin[endTfPosReferenceX];
	end_y = tfY_begin[endTfPosReferenceY];
	
	
	
	
	//ready
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("ar3");
	group.setPlanningTime(5.0);
  
	//Set maximum speed
	group.setMaxVelocityScalingFactor(0.5);

	//Set maximum acceleration
	group.setMaxAccelerationScalingFactor(0.5);
	
	
	tf2::Quaternion orientation;
	orientation.setRPY(tau / 2, 0, -tau / 2);//vertical pick
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation = tf2::toMsg(orientation);
	target_pose1.position.x = 0.0;
	target_pose1.position.y = -0.15;
	target_pose1.position.z = 0.45;
	group.setPoseTarget(target_pose1);
	group.move();
	  
	moveit::planning_interface::MoveGroupInterface group2("gripper");
	const robot_state::JointModelGroup* joint_model_group =
	group2.getCurrentState()->getJointModelGroup("gripper");
	moveit::core::RobotStatePtr current_state = group2.getCurrentState();
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
	joint_group_positions[0] = 0.04;  // 0.01
	joint_group_positions[1] = 0.04;  // 0.01
	group2.setJointValueTarget(joint_group_positions);
	group2.move();
	
	
	
	
	//add collision
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	addCollisionChessboard(collision_objects, planning_scene_interface);
	addCollisionPiece("target", object_x,object_y, piecePositionZ, collision_objects, planning_scene_interface); 
	addAllPieceOnBoard(beginPosUCI, collision_objects, planning_scene_interface);
	
	

	
	
	
	
	//transit
	aimWithCheck(object_x, object_y, object_z, Open, planning_scene_interface);//pursue retrieve distance 0.110
	
	
	//pick
	test_constraints.orientation_constraints.push_back(ocm);
	aim(object_x, object_y, object_z , Open, planning_scene_interface);
	ros::Duration( 3.0).sleep();
	aim(object_x, object_y, object_z , Close, planning_scene_interface);
	//attachObject
	ROS_INFO_NAMED("ar3_arm_pick_place", "Attach the object to Cobot");
	group2.attachObject("targetHead", "ar3_hand");
	group2.attachObject("targetBody", "ar3_hand");
	//pick
	ros::Duration( 3.0).sleep();
	aimWithCheck(object_x, object_y, object_z, Close, planning_scene_interface);
	
	//transit
	test_constraints.orientation_constraints.resize(0);
	aimWithCheck(end_x, end_y, end_z, Close, planning_scene_interface);
	
	//place
	test_constraints.orientation_constraints.push_back(ocm);
	aim(end_x, end_y,  end_z, Close, planning_scene_interface);
	ros::Duration( 3.0).sleep();
	aim(end_x, end_y,  end_z, Open, planning_scene_interface);
	//detachObject
	ROS_INFO_NAMED("ar3_arm_pick_place", "Detach the object from Cobot");
	group2.detachObject("targetHead");
	group2.detachObject("targetBody");
	//place
	ros::Duration( 3.0).sleep();
	aimWithCheck(end_x, end_y,  end_z, Open, planning_scene_interface);
	
	//ready
	group.setPoseTarget(target_pose1);
	group.move();
	
	
	ROS_INFO_NAMED("ar3_arm_pick_place", "Remove the objects from the world");
	std::vector<std::string> object_ids;
	for(int i = 0; i < collision_objects.size(); i++)
	{
		object_ids.push_back(collision_objects[i].id);
	}
	planning_scene_interface.removeCollisionObjects(object_ids);
}


void manipulate(std::string instruction)
{
	  //ros::init(argc, argv, "panda_arm_pick_place");
	  //ros::NodeHandle nh;
	  //ros::AsyncSpinner spinner(1);
	  //spinner.start();

	  //ros::WallDuration(1.0).sleep();
	  
	  if(instruction[0] == 'x')
	  {
	  	ROS_INFO("%s",instruction.substr(1,4).c_str());
	  	uci2tfPickPlace( instruction.substr(1,4));
	  	ROS_INFO("%s",instruction.substr(5,4).c_str());
	  	uci2tfPickPlace( instruction.substr(5,4));
	  }
	  else
	  {
	  	ROS_INFO("%s",instruction.c_str());
	  	uci2tfPickPlace( instruction);
	  }
  

}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	char buffer[20];
	int i = 0;
	while(msg->data.c_str()[i] != '\0')
	{
		buffer[i] = msg->data.c_str()[i];
		i++;
	}
	buffer[i] = '\0';
	std::string instruction(buffer);
	implementation = instruction;
}

void chatterCallbackChessMonitor(const std_msgs::String::ConstPtr& msg)
{
	char display[49];
	static long long iteration = 0;

	
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
	}
	
	
	if(iteration / 9 % 10 == 9)
	{
		//system("clear");
	}
	iteration++;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ar3_arm_pick_place");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("instruction_from_chessEngine", 1000, chatterCallback);
	ros::Subscriber sub2Chessboard = nh.subscribe("msgs_from_sensor_chessboard", 1000, chatterCallbackChessMonitor);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Rate rate(1.0);
	
	
	double begin =ros::Time::now().toSec(), secs;
	
	
	
	//ros::Duration( 2.0).sleep();//waiting for the activate of the broadcaster
	//ros::Duration( 10.0).sleep();
	while(nh.ok())
	{
		if(implementation != "none")
		{
			manipulate(implementation);
			implementation = "none";
		}
		rate.sleep();
		//ros::spinOnce();
	}
	return 0;
}


