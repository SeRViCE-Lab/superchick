/*
*    This code reads the vicon markers when vicon is turned on by passing the 
*    handle sim:=false in vicon launch or via the argv arguments if running 
*    with rosrun.
*
*
*	 When vicon is off and we are simulating the vicon clouds, we can pass the 
*	 sim argument as true
*
*	Author: Olalekan Ogunmolu
*   Date: October 28, 2016
*   
*   Lab Affiliation: Gans' Lab, UT Dallas
*/



#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread/thread.hpp>

#include <vicon_bridge/Markers.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <stdio.h>
#include <string>
#include <vector>
#include <time.h>
#include <sstream>

class makeClouds
{
public:
	makeClouds(ros::NodeHandle nv)
	: div(1000), nv_(nv), save(false)
	{
	}

	~makeClouds()
	{	}

	void callback(const geometry_msgs::TransformStamped& twist);
	void markers_cb(const vicon_bridge::Markers& vmarkers);
	void headTwist_cb(const geometry_msgs::Twist& pose);
	void createCloud();
	void publishClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);
	void broadcast_transform(const geometry_msgs::Twist& pose);
	void lookup_transform();
	void saveCloud();
	inline void keyboardEvent(char key_sym, void *);
	void placeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr);

private:
	geometry_msgs::Point fore_marker, left_marker, right_marker, chin_marker;	
	pcl::PointXYZRGB fore_point, left_point, right_point, chin_point, red_point;
	int div;
	using PointRGB = pcl::PointCloud<pcl::PointXYZRGB>;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
	ros::NodeHandle nv_;
	ros::Publisher pub;

	tf::TransformListener listener;
    tf::StampedTransform transform;
    std::ostringstream oss;
    pcl::PCDWriter writer;
    size_t frame;
    bool save;
};

void makeClouds::callback(const geometry_msgs::TransformStamped& transform)
{
	// std::cout << "head transform: " << transform << std::endl;

	// this->keyboardEvent(&makeClouds::publishClouds, *this)
}

void makeClouds::headTwist_cb(const geometry_msgs::Twist& pose)
{

	// broadcast_transform(pose);
	lookup_transform();
}

void makeClouds::lookup_transform()
{
	if (nv_.ok())
	{
	  try{
	  	// listener.transformPointCloud("world", ros::Time(0), "/vicon/Superdude/head")
	    listener.lookupTransform("/world", "/vicon/Superdude/head",  
	                             ros::Time(0), transform);
	  }
	  catch (tf::TransformException ex){
	    ROS_ERROR("%s",ex.what());
	    ros::Duration(1.0).sleep();
	  }
	}
}

void makeClouds::broadcast_transform(const geometry_msgs::Twist& pose)
{
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(pose.linear.x, pose.linear.y, pose.linear.z) );
		tf::Quaternion q;
		q.setRPY(pose.angular.x,pose.angular.y, pose.angular.z);
		transform.setRotation(q);
	  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/vicon/Superdude/head"));
}

void makeClouds::markers_cb(const vicon_bridge::Markers& vmarkers)
{
	// ROS_INFO_STREAM("head markers: \n" << vmarkers);
	fore_marker 	= vmarkers.markers[0].translation;
	chin_marker 	= vmarkers.markers[1].translation;
	right_marker 	= vmarkers.markers[2].translation;
	left_marker 	= vmarkers.markers[3].translation;

	this->createCloud();
}

void makeClouds::createCloud()
{
	uint8_t r(255), g(15), b(15);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	fore_point.x = fore_marker.x/1000; fore_point.y = fore_marker.y/1000; fore_point.z = fore_marker.z/1000;
	chin_point.x = chin_marker.x/1000; chin_point.y = chin_marker.y/1000; chin_point.z = chin_marker.z/1000;
	right_point.x= right_marker.x/1000; right_point.y = right_marker.y/1000; right_point.z = right_marker.z/1000;
	left_point.x  = left_marker.x/1000; left_point.y = left_marker.y/1000; left_point.z = left_marker.z/1000;

	uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
	              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      fore_point.rgb = *reinterpret_cast<float*>(&rgb);
      left_point.rgb = *reinterpret_cast<float*>(&rgb);
      right_point.rgb = *reinterpret_cast<float*>(&rgb);
      chin_point.rgb = *reinterpret_cast<float*>(&rgb); 
            								 

      cloud_ptr->points.push_back (fore_point);
      cloud_ptr->points.push_back (left_point);
      cloud_ptr->points.push_back (right_point);
      cloud_ptr->points.push_back (chin_point);

      if (fore_point.z < 0.0)
      {
        r -= 12;
        g += 12;
      }
      else
      {
        g -= 12;
        b += 12;
      }
     cloud_ptr->width = (int) cloud_ptr->points.size();
     cloud_ptr->height = 1;

     placeCloud(cloud_ptr);
}

inline void makeClouds::placeCloud(PointRGB::Ptr cloud_ptr)
{	
 //rotate the cloud to fit head in move_it
 float theta = -M_PI/2;
 Eigen::Affine3f cloud_transformer = Eigen::Affine3f::Identity();
 cloud_transformer.translation() << -0.6, 0.5, 0.1;
 //rotate around z axis
 cloud_transformer.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
 // Execute the transformation
 PointRGB::Ptr transformed_cloud (new PointRGB ());
	 // You can either apply transform_1 or transform_2; they are the same
 pcl::transformPointCloud (*cloud_ptr, *transformed_cloud, cloud_transformer);

 this->cloud_ptr = transformed_cloud;
 publishClouds(this->cloud_ptr);
}

void makeClouds::publishClouds(PointRGB::Ptr cloud_ptr)
{		
	sensor_msgs::PointCloud2 pcl_msg;
	pcl::toROSMsg(*cloud_ptr, pcl_msg);
	pcl_msg.header.stamp = ros::Time::now();
	pcl_msg.header.frame_id = "/vicon_cloud_frame";

	pub = nv_.advertise<sensor_msgs::PointCloud2>("/vicon/clouds", 1);
	pub.publish(pcl_msg);

	 if(save)
		this->saveCloud();

	ros::Rate loop_rate(30); 	
	loop_rate.sleep();

}

inline void makeClouds::keyboardEvent(char key_sym, void *) 
{
	if(key_sym =='s')
		save = true;
}

void makeClouds::saveCloud() 
{
	oss.str("");
	oss << "./" << std::setfill('0') << std::setw(4) << frame;
	const std::string basename = oss.str();
	const std::string cloudname = basename + "_cloud.pcd";

	ROS_INFO_STREAM("saving cloud, " << cloudname);
	writer.writeBinary(cloudname, *(this->cloud_ptr));
	++frame;
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////GLOBAL PROTOTYPES & FUNCIONS///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
static bool getROSPackagePath(const std::string pkgName, boost::filesystem::path & pkgPath);
static std::string getDateTimeStr();

static std::string getDateTimeStr()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, 80, "%F.%T", timeinfo);
    return (buffer);
}

static bool getROSPackagePath(const std::string pkgName, boost::filesystem::path & pkgPath)
{
    pkgPath = ros::package::getPath(pkgName);
    if (pkgPath.empty())
    {
        ROS_WARN("Could not find package '%s' ", pkgName.c_str());
        return false;
    }
    else
    {
        ROS_INFO("%s package found here: %s", pkgName.c_str(), pkgPath.c_str());
        return true;
    }
}
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle nv; 
	bool sim;

	ROS_INFO("%s started.", ros::this_node::getName().c_str());
	nv.getParam("/superchick_cloud/sim", sim);   //check if param was set on server with roslaunch
	std::cout <<  "sim: " << sim << std::endl /* << std::endl*/;


	if(sim)
	{
		// pcl::PCDGrabberBase& grabber;
		boost::filesystem::path cloud_pkg_path;
		getROSPackagePath("superchick_cloud", cloud_pkg_path);
		ROS_INFO("superchick_pkg_path: %s", cloud_pkg_path.c_str()); 
		// pcl::PCDGrabberBase::PCDGrabberImpl::PCDGrabberImpl (grabber;, const std::string& pcd_path, float frames_per_second, bool repeat);
	}	
	else  // broadcast saved tarball clouds iteratively
	{
		makeClouds mc(nv);

		ros::Subscriber sub = nv.subscribe("/vicon/Superdude/head", 1, &makeClouds::callback, &mc);	
		ros::Subscriber msub = nv.subscribe("/vicon/markers", 1, &makeClouds::markers_cb, &mc);
		ros::Subscriber htsub = nv.subscribe("/vicon/headtwist", 1, &makeClouds::headTwist_cb, &mc);		
	}

	ros::spin();

	return 0;
}

