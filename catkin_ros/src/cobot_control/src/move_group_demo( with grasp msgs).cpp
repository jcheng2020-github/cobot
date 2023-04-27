//<description>MoveIt script for chess_player_motion_series test MOVEIT P&P SOLUTION TEST move_group_demo( with grasp msgs)1 </description>
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

enum GripperInstruction{Open, Close};



// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


//object postion parameters
//hortizontal with no rotation range object_x: [-0.4 , 0.4] object_y: [-0.55, -0.65] division value: 0.003
//vetical range object_x, end_x: [-0.2 , 0.2] object_y, end_y: [ -0.50 , -0.18] division value: 0.003
double object_x = -0.20, object_y = -0.00 , object_z = 0.110;// object_z = 0.035
double moveit_deviation_x = 0.000 ,moveit_deviation_y = -0.000, moveit_deviation_z = 0.0;

double end_x =  0, end_y =-0.25, end_z = 0.110;//end_z =0.035

const double tfXOrigin_begin = 0.157, 
	dX1_begin = 0.052 , 
	dX2_begin = 0.052;
const double tfYOrigin_begin = -0.201, 
	dY1_begin = 0.044, 
	dY2_begin = 0.038;
	
const double tfXOrigin_end = 0.157, 
	dX1_end = 0.052 , 
	dX2_end = 0.052;
const double tfYOrigin_end = -0.201, 
	dY1_end = 0.038, 
	dY2_end = 0.038;



double tfX_begin[9] = { tfXOrigin_begin ,
	          tfXOrigin_begin-dX1_begin/*0.115*/,
		 tfXOrigin_begin - dX1_begin *2 , 
		 tfXOrigin_begin - dX1_begin *3, 
		 tfXOrigin_begin - dX1_begin *4,
		 tfXOrigin_begin - dX1_begin *5,
		 tfXOrigin_begin - dX1_begin *6,
		 tfXOrigin_begin - dX1_begin *7,
		 tfXOrigin_begin - dX1_begin *8};
double tfY_begin[8] = {tfYOrigin_begin ,
		 tfYOrigin_begin -  dY1_begin/*-0.253*/,
		 tfYOrigin_begin - dY1_begin *2,
		 tfYOrigin_begin - dY1_begin *3, 
		 tfYOrigin_begin - dY1_begin *4, 
		 tfYOrigin_begin - dY1_begin *5,
		 tfYOrigin_begin - dY1_begin *6,
		 tfYOrigin_begin - dY1_begin *7};
		 
std::string implementation = "none";




void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "ar3_finger_joint1";
  posture.joint_names[1] = "ar3_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "ar3_finger_joint1";
  posture.joint_names[1] = "ar3_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.010;
  posture.points[0].positions[1] = 0.010;
  posture.points[0].time_from_start = ros::Duration(5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  //grasps[0].grasp_pose.header.frame_id = "panda_link0";
  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  
  
  
  
  
  
  

  //orientation.setRPY(tau / 4, -tau / 2, -tau / 4);//horizontal with rotation pick
  //orientation.setRPY(tau / 4, -tau / 2, 0);//horizontal without rotation pick
  orientation.setRPY(tau / 2, 0, -tau / 2);//vertical pick
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  /*rotation horizontal pick
  grasps[0].grasp_pose.pose.position.x = object_x + 0.22;
  grasps[0].grasp_pose.pose.position.y = object_y;
  rasps[0].grasp_pose.pose.position.z = object_z+deviation_z + 0.044;
  */
  /*non-rotation horizontal pick
  grasps[0].grasp_pose.pose.position.x = object_x+deviation_x;
  grasps[0].grasp_pose.pose.position.y = object_y+deviation_y + 0.22;
  rasps[0].grasp_pose.pose.position.z = object_z+deviation_z + 0.044;
  */
  //verticle pick
  grasps[0].grasp_pose.pose.position.x = object_x+moveit_deviation_x;
  grasps[0].grasp_pose.pose.position.y = object_y+moveit_deviation_y;
  grasps[0].grasp_pose.pose.position.z = object_z+moveit_deviation_z + 0.255;










  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  //grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  //grasps[0].pre_grasp_approach.direction.vector.x = -1.0;
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.005;
  grasps[0].pre_grasp_approach.desired_distance = 0.095;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  //grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.05;
  grasps[0].post_grasp_retreat.desired_distance = 0.110;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  //place_location[0].place_pose.header.frame_id = "panda_link0";
  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  //orientation.setRPY( 0 , 0, tau / 4);  // A quarter turn about the z-axis
  orientation.setRPY( 0 , 0, 0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  
  
  
  
  
  
  
  

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = end_x;
  place_location[0].place_pose.pose.position.y = end_y;
  //place_location[0].place_pose.pose.position.z = 0.046;
  place_location[0].place_pose.pose.position.z = end_z;
  









  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  //place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.005;
  place_location[0].pre_place_approach.desired_distance = 0.095;


  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  //place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as negative y axis */
  //place_location[0].post_place_retreat.direction.vector.y = 0.5;
  place_location[0].post_place_retreat.direction.vector.z = 0.5;
  place_location[0].post_place_retreat.min_distance = 0.001;
  place_location[0].post_place_retreat.desired_distance = 0.110;
  
  

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  //collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.017;
  collision_objects[0].primitives[0].dimensions[1] = 0.017;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;









  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = object_x;
  collision_objects[0].primitive_poses[0].position.y = object_y;
  collision_objects[0].primitive_poses[0].position.z = -0.05;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;











  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  //collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].header.frame_id = "base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.017;
  collision_objects[1].primitives[0].dimensions[1] = 0.017;
  collision_objects[1].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = end_x;
  collision_objects[1].primitive_poses[0].position.y = end_y;
  collision_objects[1].primitive_poses[0].position.z = -0.05;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;




  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  //collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.017;
  collision_objects[2].primitives[0].dimensions[1] = 0.017;
  collision_objects[2].primitives[0].dimensions[2] = 0.089;
  
  
  
  
  
  
  

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = object_x;
  collision_objects[2].primitive_poses[0].position.y = object_y;
  collision_objects[2].primitive_poses[0].position.z = object_z;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  
  
  
  
  
  
  
  
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}



void aim(double x, double y, double z, GripperInstruction clawInstruction)
{
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit::planning_interface::MoveGroupInterface group("ar3");
	group.setPlanningTime(50.0);
	
	
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
		joint_group_positions[0] = 0.01;
		joint_group_positions[1] = 0.01;
	}
	group2.setJointValueTarget(joint_group_positions);
	group2.move();
	
}

void uci2tfPickPlace(std::string uci)
{
	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "ar3_hand";
	ocm.header.frame_id = "base_link";
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.01;
	ocm.absolute_y_axis_tolerance = 0.01;
	ocm.weight = 1.0;
	moveit_msgs::Constraints test_constraints;

	double object_x;
	double object_y;
	double object_z = 0.360 ;
	double end_x;
	double end_y;
	double end_z = 0.360 ;
	
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
	group.setPlanningTime(50.0);
  
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
	
	//transit
	aim(object_x, object_y, object_z  + 0.110, Open);//retrieve distance 0.110
	
	
	//pick
	test_constraints.orientation_constraints.push_back(ocm);
	aim(object_x, object_y, object_z , Open);
	ros::Duration( 2.0).sleep();
	aim(object_x, object_y, object_z , Close);
	ros::Duration( 2.0).sleep();
	aim(object_x, object_y, object_z  + 0.110, Close);
	
	//transit
	test_constraints.orientation_constraints.resize(0);
	aim(end_x, end_y, end_z + 0.110, Close);
	
	//place
	test_constraints.orientation_constraints.push_back(ocm);
	aim(end_x, end_y,  end_z, Close);
	ros::Duration( 2.0).sleep();
	aim(end_x, end_y,  end_z, Open);
	ros::Duration( 2.0).sleep();
	aim(end_x, end_y,  end_z + 0.110, Open);
	
	//ready
	group.setPoseTarget(target_pose1);
	group.move();
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
	  //uci2tfPickPlace("a3b1");
	
	



	

	
	
	
	
		/*
	//Attaching objects to the arm
	std::vector<moveit_msgs::CollisionObject> object_to_attach;
	object_to_attach.resize(1);
	object_to_attach[0].id = "piece";
	

	object_to_attach[0].header.frame_id = "base_link";
	
	object_to_attach[0].primitives.resize(1);
	object_to_attach[0].primitives[0].type = object_to_attach[0].primitives[0].BOX;
	object_to_attach[0].primitives[0].dimensions.resize(3);
	object_to_attach[0].primitives[0].dimensions[0] = 0.017;
	object_to_attach[0].primitives[0].dimensions[1] = 0.017;
	object_to_attach[0].primitives[0].dimensions[2] = 0.089;
	
	object_to_attach[0].primitive_poses.resize(1);
	object_to_attach[0].primitive_poses[0].position.x = tfXOrigin_begin;
  	object_to_attach[0].primitive_poses[0].position.y = tfYOrigin_begin;
	object_to_attach[0].primitive_poses[0].position.z =  0.035;
	object_to_attach[0].primitive_poses[0].orientation.w = 1.0;
	
	object_to_attach[0].operation = object_to_attach[0].ADD;
	planning_scene_interface.applyCollisionObjects(object_to_attach);
	group2.attachObject(object_to_attach[0].id, "ar3_hand");
	*/
	/*
	//Detaching and Removing Objects
	group2.detachObject(object_to_attach[0].id);
	
	//remove the objects
	std::vector<std::string> object_ids;
	object_ids.push_back(object_to_attach[0].id);
	planning_scene_interface.removeCollisionObjects(object_ids);
	*/

  
  /*
 
  
  

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(4.0).sleep();

  pick(group);

  ros::WallDuration(4.0).sleep();

  place(group);
  
  
  
    //Set maximum speed

    group.setMaxVelocityScalingFactor(0.5);

    //Set maximum acceleration

    group.setMaxAccelerationScalingFactor(0.5);
  orientation.setRPY(tau / 2, 0, -tau / 2);//vertical pick
  target_pose1.orientation = tf2::toMsg(orientation);
  target_pose1.position.x = 0.0;
  target_pose1.position.y = -0.15;
  target_pose1.position.z = 0.45;
  group.setPoseTarget(target_pose1);
  group.move();
*/
  

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

int main(int argc, char **argv)
{
	/*
	//coordinate adjustment
	tfX[0] = 0.165; 
	tfY[0] = -0.200;
	for(int i = 0; i < 8; i++)
	{
		tfX[i] = tfX[0] - 0.055 * i;
		tfY[i] = tfY[0] - 0.056 * i;
	}
	*/

	ros::init(argc, argv, "ar3_arm_pick_place");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("instruction_from_chessEngine", 1000, chatterCallback);
	
	ros::AsyncSpinner spinner(1);
	spinner.start();
	/*
	ros::WallDuration(1.0).sleep();
	
	
	

	tf::TransformListener listener;
	*/
	ros::Rate rate(1.0);
	
	
	double begin =ros::Time::now().toSec(), secs;
	
	
	
	//ros::Duration( 2.0).sleep();//waiting for the activate of the broadcaster
	//ros::Duration( 10.0).sleep();
	while(nh.ok())
	{
		/*
		secs =ros::Time::now().toSec();
		ROS_INFO("%f", secs - begin);
		if(secs - begin > 20)
		{
			ROS_INFO("void manipulate() begin");
			manipulate();
			begin =ros::Time::now().toSec();
		}
	
	
	
		tf::StampedTransform transform;
		try
		{
			listener.waitForTransform("potential_target/bottle_0", "base_link",
                              ros::Time::now(), ros::Duration(1.0));
			listener.lookupTransform( "potential_target/bottle_0", "base_link",
					ros::Time(0), transform);
		}
		catch( tf::TransformException ex)
		{
			//ROS_ERROR( "%s", ex.what());
			ROS_INFO("Waiting for target position data update......");
			ros::Duration( 1.0).sleep();
			continue;
		}
		
		object_x = -transform.getOrigin().x();
		object_y = -transform.getOrigin().y();
		object_z = -transform.getOrigin().z();
		
		detected_x = -transform.getOrigin().x();
		detected_y = -transform.getOrigin().y();
		detected_z = -transform.getOrigin().z();

		ROS_INFO("\ndetected_x: %s\ndetected_y: %s\ndetected_z: %s\n\n", 
		std::to_string(detected_x ).c_str(),
		std::to_string(detected_y ).c_str(),
		std::to_string(detected_z ).c_str()
		);

		rate.sleep();
		*/
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


