<!-- ### This is the mujoco binding to the osrf ros code from 2016 -->

### SOFA Plugin

There is now a SOFA scene file. Here, we arranged three soft robot kinematic chains around the head of the patient (there are eight individual soft robots), position the patient lying in a supine manner on a treatment table, and then actuate the soft robots to compensate for patient motion. The soft robots are interconnected by passive elastic materials to allow motion contact with the head throughout manipulation. Control details are in my PhD dissertation.

Example screenshots are presented below:

![Full scene file](/IAB/patient/data/Full-scene.png)

![Head and Soft Robots only](/IAB/patient/data/Head-Soros.png)

Source codes for this scene file are in

+ [CPP Source File](/IAB8/src/IAB.cpp)  and

+ [Scene Main File](/IAB8/scenes/imrt.scn)

### MUJOCO Plugin

1. 	Install [openscene-graph](http://www.openscenegraph.org/): `sudo apt-get install openscenegraph libopenscenegraph-dev`.

2. Install [mujoco](https://www.roboti.us/index.html) (v1.50+) and then put the downloaded mjpro150 folder in your home directory under a folder called mujoco. MuJoCo is a high-quality physics engine and requires a license to use. Obtain a key, which should be named mjkey.txt, and place the key into the mjpro150 directory.

3. Compile the mujoco model viewer with catkin: `catkin build`

4. Visualize the model: `rosrun superchick chick_viewer`.

	Below is an example of the scene in mjpro150:

	<!-- ![mjpro_model](/superchick/config/mjmodel.jpg) -->

	<div class="fig figcenter fighighlight">
	  <img src="/superchick/config/mjmodel.jpg" height="75%" width="70%" align="middle" style="border-left: 1px solid black;" />
	</div>

### Superchicko Rviz Plugin

Generate superchick urdf like `rosrun xacro xacro src/superchicko/superchick/superchick.xacro > superchick.xacro`

### Novalis Rviz Plugin
Then run urdf:	`roslaunch superchick display.launch model:=superchick.urdf`

run novalis like: `roslaunch superchick display.launch model:=src/superchicko/superchick/novalis.xacro`

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
```
