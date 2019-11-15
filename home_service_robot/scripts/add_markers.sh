#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE="$PWD/src/home_service_robot/worlds/single-floor.world"

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$PWD/src/home_service_robot/maps/single_floor.yaml" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " rosrun add_markers add_markers_node"

