ispl### Superchicko Rviz Plugin

Generate superchick urdf like 

```bash
rosrun xacro xacro src/superchicko/superchick/superchick.xacro > superchick.xacro
```

### Novalis Rviz Plugin
Then run urdf like longhorn cattle, aka

```bash
	roslaunch superchick display.launch model:=superchick.urdf
```

run novalis like

```bash
	roslaunch superchick display.launch model:=src/superchicko/superchick/novalis.xacro
```

### Example Display in Rviz
![Example Display in Rviz](/superchick/meshes/model.png)

### Planning works in MoveIt! 

run moveit! like:

```bash
	roslaunch supermove_config demo.launch 
```
If you run into runtime errors, be sure to install  `ros-<distro>-moveit-ros-visualization` from the ubuntu repos

![Example Display in Moveit!](/superchick/meshes/moveit!.png)

To run simulation of head in move it:

```bash
	roslaunch supermove_config moveit_planning_execution.launch 
```

### Point CLouds from Vicon 

Written in `rospy`, this transforms each of the markers placed on superchick/Superdude into an array of four points (since we use four markers).

The twist of the head, as computed in vicon_listener, is used to generate a `geometry_msgs/TransformStamped` Object. The topic the Superdude markers are published
on (e.g. /vicon/Superdude/root) contains the twist info. Since `/vicon/markers` contains the translation of each marker in the world, I used the `/vicon/markers` points to generate the clouds and compute the directions of the point cloud objects with respect to the twist information about the center of the face.
The result is published as `/vicon_clouds` for onward streaming into `move-it` for real-time control. Below is an example cloud from the four points on the head transformed into a `sensor_msgs/PointCloud2` object.

![Vicon Clouds](/superchick_cloud/clouds.png)


### @TODO:
To deploy on real robot, pass arg sim as true. This will launch the `superchick/launch/robot_interface.launch` file that brings the robot alive.

### Changelog

- Added moveit demo run instructions to readme Sept 28, 10:44pm (LKN)
- Added cloud info to readme (Oct 6)


### Known Issues

#### Having issues importing rospkg 
When launching the xacro generated urdf files in superchicko, the system complains it cannot find `rospkg`?
This is because Anacoda installs nmesses up the PYTHONPATH variable so that even if rospkg is listed in `/usr/lib/python/distpackages`, the ros interpreter doesn't know which version of python to use. One way to solve this is to comment out the python path that is automatically imported in bash (didn't work for me for some reason) or to manually reinstall python and associated dependedncies e.g. wxPython and then manually export the installation path to you r LD_LIBRARY_PATH

```bash
	export LD_LIBRARY_PATH=/home/robotec/Documents/wxPython-src-3.0.2.0/bld/lib:/usr/local/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH

	export PYTHONPATH=/home/robotec/Documents/wxPython-src-3.0.2.0/wxPython:$PYTHONPATH
```

#### libompl.so issues
When having issues with loading libompl.so.10 with movegroup node, doing the following after installing ompl with the bash script from bitbucket solved my problem on the robotec machine.

```bash
	sudo cp /opt/ros/indigo/lib/x86_64-linux-gnu/libompl.so.10 /usr/local/lib/x86_64-linux-gnu/