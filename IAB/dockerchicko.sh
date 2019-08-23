#! /bin/bash
#
# export SOFA_ROOT=/sofa/build
export IMAGE=lakehanne/sofa:IAB

docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    -v ~/Documents/superchicko:/root/superchicko:rw \
    $IMAGE
