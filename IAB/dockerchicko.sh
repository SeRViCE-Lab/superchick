#! /bin/bash
xhost +local:root
# export SOFA_ROOT=/sofa/build
export IMAGE=lakehanne/sofa:IAB

if [[ "$(uname -s)" == 'Linux'  ]]; then
    echo -e "OS: Linux. Running docker-run\n"
    docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -e DISPLAY=$DISPLAY \
      -v ~/Documents/superchicko:/root/superchicko:rw \
      $IMAGE
elif [[ "$(uname -s)" == 'Darwin' ]]; then
  #statements
  # echo -e "OS: MAC.\n"
  # # docker-compose up
  # docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  #   -e DISPLAY=$DISPLAY \
  #   -v ~/Documents/superchicko:/root/superchicko:rw \
  #   $IMAGE
  export ip=$(ifconfig en1 | grep inet | awk '$1=="inet" {print $2}')
  # echo -e "ip: ${ip}"
  xhost + $ip
  docker run --privileged -it --rm -e DISPLAY=${ip}:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ~/Documents/superchicko:/root/superchicko lakehanne/sofa:IAB
  # docker run --privileged -it --rm -e DISPLAY=${ip}:0 -v /tmp/.X11-unix:/tmp/.X11-unix \
  #       -v ~/Documents/superchicko:/root/superchicko jess/firefox
  # docker run -d --name IAB DISPLAY=$ip:0 -v /tmp/.X11-unix:/tmp/.X11-unix ~/Documents/superchicko:/opt/ros_ws/ lakehanne/sofa:IAB

fi

# remove x from access control list
# xhost -local:root
# xhost -$ip
# QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
# The constructor with a QGLFormat is deprecated, use the regular contructor instead.
# libGL error: No matching fbConfigs or visuals found
# libGL error: failed to load driver: swrast
# QOpenGLWidget: Failed to create context
# QOpenGLWidget: Failed to create context
# QOpenGLWidget: Failed to create context
