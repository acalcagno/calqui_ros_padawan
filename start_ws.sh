#! /bin/bash
SCRIPT_PATH=$(dirname "$(readlink -f "$0")")

echo $(readlink -f "$0")

echo $SCRIPT_PATH

xhost +
docker run --privileged --rm \
       -e DISPLAY=${DISPLAY} \
       --device=/dev/video0:/dev/video0 \
       --device=/dev/dri:/dev/dri \
       --device=/dev/ttyACM0 \
       -v $SCRIPT_PATH:$SCRIPT_PATH \
       -v /home/agustin/.ssh:/home/agustin/.ssh \
       -v /tmp/.X11-unix:/tmp/.X11-unix:ro $@ \
       --gpus all \
       -it agu_ros_padawan /bin/bash
