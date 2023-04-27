//<description>tf broadcaster for the reference object of arm</description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

std::string object_name[2];


int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_tf_broadcaster");
	if(argc != 1 + 7 + 7)
	{
		ROS_ERROR("Invalid number of parameters\nusage: object_tf_broadcaster child_frame_name x y z roll pitch yaw");
	}
	object_name[0] = argv[1];
	object_name[1] = argv[8];
	
	ros::NodeHandle node;
	ros::Rate rate(10.0);
	
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	
	while (node.ok())
	{
		transform.setOrigin( tf::Vector3(atof(argv[2]), atof(argv[3]), atof(argv[4])) );
		q.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
		transform.setRotation(q);
		br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "camera_link", object_name[0]));
		
		transform.setOrigin( tf::Vector3(atof(argv[9]), atof(argv[10]), atof(argv[11])) );
		q.setRPY(atof(argv[12]), atof(argv[13]), atof(argv[14]));
		transform.setRotation(q);
		br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "potential_target/remote_0", object_name[1]));
		
		//ROS_INFO("Spinning until killed publishing %s and %s tf transform", object_name[0].c_str(), object_name[1].c_str());
		rate.sleep();
	}
	
	return 0;
}
