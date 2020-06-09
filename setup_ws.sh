#!/usr/bin/env bash

echo sourcing /opt/ros/melodic/setup.bash
source /opt/ros/melodic/setup.bash

echo sourcing catkin_ws/devel/setup.bash

echo exporting variable TURTLEBOT3_MODEL=burger
export TURTLEBOT3_MODEL=burger



# tmux
tmux start-server
tmux new-session -d -s padawan


# RUN roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# Screen 0: gazebo
tmux rename-window -t padawan:0 gazebo
tmux send-keys -t padawan:0 "roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch" C-m

# Screen 1: shell
tmux new-window -t padawan:1
tmux rename-window -t padawan:1 'shell'
tmux send-keys -t padawan:1 "source catkin_ws/devel/setup.bash" C-m
tmux send-keys -t padawan:1 "rosrun calqui goto_server.py" C-m

source /opt/ros/melodic/setup.bash

tmux split-window -t padawan:1 -v -p 50

# attach
tmux attach
