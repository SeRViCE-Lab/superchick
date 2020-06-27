### Introduction

This implements the Tetrahedron Mooney-Rivlin Plugin for the half-domes used in head positioning.

A self contained [python](/sofa/python) and [C++](/sofa/src) implementation are available to the user. To compile the C++ package, cd to the build subfolder in sofa directory and run the bash file [compile](/sofa/compile).


### RUNNING

In your docker environment, type `sup` into terminal. This should take you to the superchick directory. When there, `cd sofa/build` and run the `compile` bash script as follows:

```bash
usage: compile [-b] [-c] [-l] [-s]
 -b [build new executables]
 -c [cleanup build directory]
 -l launch the parallel mechanism executable after building
 -s launch single IAB executable after building
 e.g. ../compile -b n -c n -l n -s y
```

For the python implementation, the generic `runSofa` executable that ships with SOFA is copied into the build directory when you run `compile`. The python scene file can be run as

`runSofa ../python/SingleIAB.py` for a single IAB or `runSofa ../python/MRILinac.py` for the MRI-LINAC RT proposed mechanism. Further details on the python implementation is in the folder [python/README.md](/sofa/python/README.md)


That's all there is to it.


### DOCKER CONFIG

+ The default entry point for this image is /opt/ros2_ws. This is the directory that contains the ros2_ws

+ The sofa installation is in /sofa.

+ To test the installation, please bind your X11 display on the host machine to the docker container's display before runninng (explained further below)
	- To keep the image light, I have packaged the image without X11 support. However, you can run SofaGUI by volume mounting your host's X11 display to the image with the following commands
          `docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v ~/Documents/superchicko:/root/superchicko:ro -e DISPLAY=$DISPLAY lakehanne/sofa:IAB`

+ It is a fork of the newly released v19.06 version alongside with SofaROSConnector, SoftRobots, and STLIB plugins

+ SofaROSConnector was packaged by Fabian Aichele and runs with ROS kinetic
	- This image's git repo is at my git fork lakehanne/sofa.

+ Note that the shell environment is Oh-my-zsh. I left a bash_aliases file in /root/.bash_aliases that .zshrc sources.
Make your custom aliases and path commands there.

### FAQs

Selecting default xcode to use for CMD Line Tools

https://developer.apple.com/library/archive/technotes/tn2339/_index.html#//apple_ref/doc/uid/DTS40014588-CH1-HOW_DO_I_SELECT_THE_DEFAULT_VERSION_OF_XCODE_TO_USE_FOR_MY_COMMAND_LINE_TOOLS


+ Mooney Rivlin ForceField Implemented Here: ~/sofa/applications/plugins/Flexible/material/MooneyRivlinForceField.h_

+ Gmsh not loading: Gmsh file conversions: When exporting from stl or obj to gmsh, be sure to choose the options "Version 2 ASCII" and "save all elements" in the options box.
