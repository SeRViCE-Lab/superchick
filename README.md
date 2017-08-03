### This is the mujoco bindings to the osrf ros code from last year

### MUJOCO Plugin

1. 	Install [openscene-grapgh](sudo apt-get install openscenegraph libopenscenegraph-dev).

2. Then install [mujoco](https://www.roboti.us/index.html) (v1.50+) and put the downloaded mjpro150 folder in your home directory under a folder called `mujoco`. MuJoCo is a high-quality physics engine and requires requires a license. Obtain a key, which should be named mjkey.txt, and place the key into the mjpro150 directory.

3. Compile the mujoco model viewer with catkin like so:

`catkin build`

4. Then visualize the model like so:

		```bash
			rosrun superchick chick_viewer
		```

### Superchicko Rviz Plugin

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
