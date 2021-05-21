#!/bin/sh

# source path might need to be modified (if the relative is not correct)!
gnome-terminal --tab -- bash -c "source ../../devel/setup.bash; export TURTLEBOT3_MODEL=burger; roslaunch launchers main_launcher.launch; exec bash"
# sleep 2
# gnome-terminal --tab -- bash -c "rviz -d multi_robot.rviz; exec bash"