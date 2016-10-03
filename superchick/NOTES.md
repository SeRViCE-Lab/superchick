
### Having issues importing rospkg 
When launching the xacro generated urdf files in superchicko, the system complains it cannot find `rospkg`?
This is because Anacoda installs nmesses up the PYTHONPATH variable so that even if rospkg is listed in `/usr/lib/python/distpackages`, the ros interpreter doesn't know which version of python to use. One way to solve this is to comment out the python path that is automatically imported in bash (didn't work for me for some reason) or to manually reinstall python and associated dependedncies e.g. wxPython and then manually export the installation path to you r LD_LIBRARY_PATH

```bash
	export LD_LIBRARY_PATH=/home/robotec/Downloads/wxPython-src-3.0.2.0/bld/lib:/usr/local/lib/x86_64-linux-gnu/:$LD_LIBRARY_PATH

	export PYTHONPATH=/home/robotec/Downloads/wxPython-src-3.0.2.0/wxPython:$PYTHONPATH
```

### libompl.so issues
When having issues with loading libompl.so.10 with movegroup node, doing the following after installing ompl with the bash script from bitbucket solved my problem on the robotec machine.

```bash
	sudo cp /opt/ros/indigo/lib/x86_64-linux-gnu/libompl.so.10 /usr/local/lib/x86_64-linux-gnu/
```

