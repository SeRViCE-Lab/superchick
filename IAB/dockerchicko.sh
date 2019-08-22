#! /bin/bash

docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    -v ~/Documents/superchicko:/root/superchicko:rw \
    lakehanne/sofa:ros2
