#! /bin/bash
SCRIPT_PATH=$(dirname "$(readlink -f "$0")")

xhost +
docker run --privileged --rm \
       -e DISPLAY=${DISPLAY} \
       --device=/dev/video0:/dev/video0 \
       --device=/dev/dri:/dev/dri \
       -v $SCRIPT_PATH:$SCRIPT_PATH \
       -v /home/agustin/.ssh:/home/agustin/.ssh \
       -v /tmp/.X11-unix:/tmp/.X11-unix:ro $@ \
       --gpus all \
       -it agu_ros_padawan /bin/bash
