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

### Example Display in Rviz
![Example Display in Rviz](/superchick/meshes/model.png)

### Planning works in MoveIt! 

run moveit! like:

```bash
	roslaunch supermove_config demo.launch 
```
If you run into runtime errors, be sure to install  `ros-<distro>-moveit-ros-visualization` from the ubuntu repos

![Example Display in Moveit!](/superchick/meshes/moveit!.png

@TODO:
To deploy on real robot, pass arg sim as true. This will launch the `superchick/launch/robot_interface.launch` file that brings the robot alive.

### Changelog

- Added moveit demo run instructions to readme Sept 28, 10:44pm (LKN)