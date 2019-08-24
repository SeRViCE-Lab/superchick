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
  echo -e "OS: MAC. Running docker compose up\n"
  # docker-compose up
  docker run -ti --rm -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=$DISPLAY \
    -v ~/Documents/superchicko:/root/superchicko:rw \
    $IMAGE
fi
