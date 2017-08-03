#include <ros/ros.h>
#include <sstream>
#include <string.h>
#include <ros/package.h>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/tuple/tuple.hpp>
#include "superchick/mujoco_osg_viewer.hpp"

namespace pathfinder
{
	bool getROSPackagePath(const std::string pkgName, boost::filesystem::path & pkgPath)
	{
	    pkgPath = ros::package::getPath(pkgName);
	    if (pkgPath.empty())
	    {
	        printf("Could not find package '%s' ", pkgName.c_str());
	        return false;
	    }
	    else
	    {
	        printf("%s package found here: %s", pkgName.c_str(), pkgPath.c_str());
	        return true;
	    }
	}

	void getMujocoFile(boost::filesystem::path & mujocoPath)
	{
		std::string const & user_name = std::getenv("USER");
		std::string const& key_path = user_name + "/mujoco/mjpro150/mjkey.txt";

		mujocoPath = "/home/" + key_path;
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

	mjOption* option;
	mjModel* model;

	char *model_filename = new char[model_filename_c.length() + 1];
	strcpy(model_filename, model_filename_c.c_str());

	// activate software
	boost::filesystem::path mjkey_file;
	pathfinder::getMujocoFile(mjkey_file);

	printf("mjkey_file: %s\n", mjkey_file.c_str());

	mj_activate(mjkey_file.c_str());

	NewModelFromXML(model_filename, model);
	mjData* data = mj_makeData(model);
	delete[] model_filename;

	if(!model)
		ROS_WARN("Ouch, had problem loading model there: %s",  model_filename);

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

    return EXIT_SUCCESS;
}
