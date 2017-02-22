
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class Receiver
{
private:
	geometry_msgs::TransformStamped transformed_msg;
	geometry_msgs::Vector3 translation;	
	geometry_msgs::Quaternion rotQuat;
	std::mutex mutex;
	bool updatePose;
	double roll, pitch, yaw;
	std::thread getRPYThread;
	std::vector<std::thread> threadsVector;
	ros::NodeHandle np_;
 	unsigned long const hardware_concurrency;
	ros::AsyncSpinner spinner;
	ros::Subscriber pose_sub_;

public:
	Receiver(ros::NodeHandle np)
	: updatePose(false), np_(np), hardware_concurrency(std::thread::hardware_concurrency()),
	spinner(hardware_concurrency/2)
	{
		// pose_sub_ = np_.subscribe("/vicon/Superdude/root", 10, &Receiver::pose_callback, this); 

	}

	~Receiver()
	{

	}

/*	void spawn()
	{	
		begin();
		end();	
	}

	void begin()
	{
		if(spinner.canStart())
		{
			spinner.start();
			ROS_INFO("spinning with %lu threads", hardware_concurrency/2);
		}

		//spawn the threads
		threadsVector.push_back(std::thread(&Receiver::getRPYFromQuaternion, this));
		std::for_each(threadsVector.begin(), threadsVector.end(), \
		              std::mem_fn(&std::thread::join)); 
	}

	void end()
	{
	  spinner.stop(); 
	}*/

	void pose_callback(const geometry_msgs::TransformStamped::ConstPtr& pose_msg)
	{
		// ROS_INFO("in callback");
		geometry_msgs::Vector3 translation = pose_msg->transform.translation;
		geometry_msgs::Quaternion rotQuat = pose_msg->transform.rotation;
		//convert to millimeters
		metersTomilli(std::move(translation));
		//convert quaternion representation to degree
		// rad2deg(std::move(rotQuat));

		double roll, pitch, yaw;
		getRPYFromQuaternion(std::move(translation), std::move(rotQuat), 
							 std::move(roll), std::move(pitch), 
							 std::move(yaw));

		// ROS_INFO("Roll, %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);

		std::lock_guard<std::mutex> lock(mutex);
		{
			this->translation 	= translation;
			this->rotQuat    	= rotQuat;
			this->roll 			= roll;
			this->pitch 		= pitch;
			this->yaw 			= yaw;
			updatePose 			= true;			
		}
	}

	void getRPYFromQuaternion(geometry_msgs::Vector3&& translation, 
							 geometry_msgs::Quaternion&& rotQuat, double&& roll,
							 double&& pitch, double&& yaw)
	{

		tf::Quaternion q(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		rad2deg(std::move(roll)); rad2deg(std::move(pitch)); rad2deg(std::move(yaw)); 

		ROS_INFO("Roll, %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);	
	}

	inline void metersTomilli(geometry_msgs::Vector3&& translation)
	{
		translation.x	*= 1000;
		translation.y 	*= 1000;
		translation.z 	*= 1000;
	}

	inline void rad2deg(double&& x)
	{
		x  *= 180/M_PI;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_retriever_node");

	ros::NodeHandle np;

	Receiver pose(np);// = new Receiver;
	// pose.spawn();

	ros::Subscriber sub = np.subscribe("/vicon/Superdude/head", 10, &Receiver::pose_callback, &pose);    

	ros::spin();

	return 1;
}