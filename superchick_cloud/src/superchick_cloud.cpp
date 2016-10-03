#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

void callback(const geometry_msgs::TransformStamped& markers)
{
	std::cout << "Here we go: " << markers.transform.translation << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle nv; 
	ros::Subscriber sub = nv.subscribe("/vicon/Superdude/root", 1, callback);

	ros::spin();

	return 0;
}

