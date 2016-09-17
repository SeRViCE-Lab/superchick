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
![Example Display in Moveit!](/superchick/meshes/moveit!.png)