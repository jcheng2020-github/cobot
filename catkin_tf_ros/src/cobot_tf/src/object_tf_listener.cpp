//<description>tf broadcaster for monitoring the tf coordinate of target</description>
//<maintainer email="jcheng2020@my.fit.edu">Junfu Cheng</maintainer>

#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_tf_listener");
	ros::NodeHandle node;

	tf::TransformListener listener;

	ros::Rate rate(10.0);
	
	
	//ros::Duration( 2.0).sleep();//waiting for the activate of the broadcaster
	while(node.ok())
	{
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
			ROS_ERROR( "%s", ex.what());
			ros::Duration( 1.0).sleep();
			continue;
		}

		ROS_INFO("\n%s\n%s\n%s\n\n", 
		std::to_string(transform.getOrigin().x()).c_str(),
		std::to_string(transform.getOrigin().y()).c_str(),
		std::to_string(transform.getOrigin().z()).c_str()
		);

		rate.sleep();
	}
	return 0;
}
