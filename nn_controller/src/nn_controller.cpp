
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include "nn_controller/nn_controller.h"

using namespace amfc_control;

//constructor
// default Constructors.
Controller::Controller(ros::NodeHandle& n, amfc_control::ActuatorType base_bladder)
{
}

//copy constructor
Controller::Controller()
{
}

// Destructor.
Controller::~Controller()
{
}

void Controller::configure_controller(OptionsMap &options)
{

}

void Controller::set_update_delay(double new_step_length)
{
}

double Controller::get_update_delay()
{
    return 1.0;
}

void Controller::reset(ros::Time update_time)
{
}

void Controller::ref_model_subscriber(const std_msgs::String::ConstPtr& ref_model_params)
{

	std::string model_params = ref_model_params->data.c_str();
	ROS_INFO_STREAM("Model Parameters: " << model_params);  	

}

void Controller::ref_model_multisub(const std_msgs::Float64MultiArray::ConstPtr& ref_model_params)
{
	// float model_params[10]
	std::vector<double> model_params = ref_model_params->data;
	std::vector<double>::iterator iter;
	for(iter = model_params.begin(); iter!=model_params.end(); ++iter)
	{
		std::cout << "\nmodel parameters are: \n" <<
					*iter << " "; 
	}
  	
  	ros::Rate r(1);
  	r.sleep();
	Eigen::VectorXf modelParamVector;

}

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "controller_node", ros::init_options::AnonymousName);
	ros::NodeHandle n;
	amfc_control::ActuatorType at = base_bladder;
	Controller c;

    ros::Subscriber sub = n.subscribe("/saved_net", 1000, &Controller::ref_model_subscriber, &c );
    ros::Subscriber sub_multi = n.subscribe("/saved_net", 1000, &Controller::ref_model_multisub, &c );

	ros::spin();

	ros::shutdown();

	return 0;
}
