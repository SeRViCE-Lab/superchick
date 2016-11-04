/*
Base class for a controller. Controllers take in sensor readings and choose the action.
*/
#pragma once

// Headers.
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <time.h>
/* ______________________________________________________________________
    *
    *   This code is part of the superchick project. 
    *  
    *
    *   Author: Olalekan Ogunmolu
    *   Date: Nov. 3, 2016
    *   Lab Affiliation: Gans' Lab, Dallas, TX
    *__________________________________________________________________________
*/

#include <ros/time.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include "nn_controller/amfc.h"
#include "nn_controller/options.h"

namespace amfc_control
{

// Forward declarations.
class Sample;
class RobotPlugin;

class Controller
{
private:

public:
    // Constructor.
    Controller(ros::NodeHandle& n, amfc_control::ActuatorType base_bladder);
    Controller();
    // Destructor.
    virtual ~Controller();
    // Update the controller (take an action).
    // virtual void update(RobotPlugin *plugin, ros::Time current_time, boost::scoped_ptr<Sample>& sample, Eigen::VectorXd &torques) = 0;
    // Configure the controller.
    virtual void configure_controller(OptionsMap &options);
    // Set update delay on the controller.
    virtual void set_update_delay(double new_step_length);
    // Get update delay on the controller.
    virtual double get_update_delay();
    // Check if controller is finished with its current task.
    // virtual bool is_finished() const = 0;
    // Reset the controller -- this is typically called when the controller is turned on.
    virtual void reset(ros::Time update_time);
    //subscribe to the reference model parameters
    virtual void ref_model_subscriber(const std_msgs::String::ConstPtr& ref_model_params);
    virtual void ref_model_multisub(const std_msgs::Float64MultiArray::ConstPtr& ref_model_params);
};

}
