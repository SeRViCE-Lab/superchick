#include <ros/ros.h>
#include <memory>
#include <sstream>
#include <string.h>
#include <ros/package.h>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/tuple/tuple.hpp>
#include "superchick/mujoco_osg_viewer.hpp"

namespace pathfinder{
	bool getROSPackagePath(const std::string pkgName, boost::filesystem::path & pkgPath)	{
	    pkgPath = ros::package::getPath(pkgName);
	    if (pkgPath.empty())	    {
	        printf("Could not find package '%s' ", pkgName.c_str());
	        return false;
	    }
	    else	    {
	        printf("%s package found here: %s", pkgName.c_str(), pkgPath.c_str());
	        return true;
	    }
	}

	bool getMujocoFile(boost::filesystem::path & mujocoPath)	{
		std::string const & user_name = std::getenv("USER");
		std::string const& key_path = user_name + "/mujoco/mjpro150/mjkey.txt";

		mujocoPath = "/home/" + key_path;

		return true;
	}
}


int main(int argc, char** argv){

	ros::init(argc, argv, "superchick_node");

	const std::string& package_name("superchick");
	boost::filesystem::path chick_path;

	if(!pathfinder::getROSPackagePath(package_name, chick_path)){
		ROS_INFO("Could not find the package %s. Please make sure you clone the %s o package",
			package_name.c_str(), package_name.c_str());
		return EXIT_FAILURE;
	}

	std::stringstream ss;
  	ss << chick_path.c_str() << "/models/superchick.mjcf";
  	std::string model_filename_c = ss.str();

	mjModel* model;

	char *model_filename = new char[model_filename_c.length() + 1];
	// char* model_filename = std::make_shared<char> (new char[model_filename_c.length() + 1]);// no_except;
	strcpy(model_filename, model_filename_c.c_str());

	// activate software
	boost::filesystem::path mjkey_file;
	if(!pathfinder::getMujocoFile(mjkey_file))
		printf("Ouch, I had problem reconciling the path variable with your dir structure. %s",
			   "please ensure you have a Linux system. Darwin architectures are yet unsupported.");

	mj_activate(mjkey_file.c_str());

	NewModelFromXML(model_filename, model);
	mjData* data = mj_makeData(model);
	delete[] model_filename;

	if(!model)
		ROS_FATAL("Ouch! I had problem loading model %s there",  model_filename);

    MujocoOSGViewer viewer;
    viewer.SetModel(model);
    viewer.SetData(data);
    viewer.Idle();

    while(!ros::ok()){
    	mj_step(model, data);
    	viewer.SetData(data);
    	viewer.RenderOnce();
    	OpenThreads::Thread::microSleep(30000);
    }
		//deallocate existing model
		mj_deleteModel(model);
		//deallocate existing mjData
		mj_deleteData(data);
    mj_deactivate();

    return EXIT_SUCCESS;
}
