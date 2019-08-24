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


FAQS:
+ Connecting a X on MAC to remote machine
