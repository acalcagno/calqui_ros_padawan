#!/usr/bin/env bash

echo sourcing /opt/ros/melodic/setup.bash
source /opt/ros/melodic/setup.bash

echo sourcing catkin_ws/devel/setup.bash
source ../devel/setup.bash

echo exporting variable TURTLEBOT3_MODEL=burger
export TURTLEBOT3_MODEL=burger

# tmux
tmux start-server
tmux new-session -d -s padawan


# RUN roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
# Screen 0: gazebo
tmux rename-window -t padawan:0 gazebo_emtpy
tmux send-keys -t padawan:0 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:0 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:0 "echo roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch" C-m

# Screen 1: shell
tmux new-window -t padawan:1
tmux rename-window -t padawan:1 'shell'
#run the app
tmux send-keys -t padawan:1 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:1 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:1 "rosrun calqui goto_server.py" C-m

tmux split-window -t padawan:1 -h -p 70
#send goal
tmux send-keys -t padawan:1.1 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:1.1 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:1.1 "echo rostopic pub /goto_place/goal" C-m

# bottom pannel
tmux split-window -t padawan:1 -v -p 50
# dynamyc recofigure
tmux send-keys -t padawan:1.2 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:1.2 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:1.2 "rosrun rqt_gui rqt_gui -s reconfigure" C-m

tmux split-window -t padawan:1 -h -p 90
# result
tmux send-keys -t padawan:1.3 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:1.3 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:1.3 "rostopic echo /goto_place/result" C-m

tmux split-window -t padawan:1 -h -p 50
# feedback
tmux send-keys -t padawan:1.4 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:1.4 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:1.4 "rostopic echo /goto_place/feedback" C-m

# Screen 2: gazebo
tmux new-window -t padawan:2
tmux rename-window -t padawan:2 'gazebo_slam'
tmux send-keys -t padawan:2 "source /opt/ros/melodic/setup.bash" C-m
tmux send-keys -t padawan:2 "source ../devel/setup.bash" C-m
tmux send-keys -t padawan:2 "roslaunch calqui calqui.launch" C-m


# attach
tmux attach
