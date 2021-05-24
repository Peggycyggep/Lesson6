#!/bin/sh 
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch add_markers RVIZ_Navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 10
xterm -e " source devel/setup.bash; rosrun add_markers marker_shape " &
sleep 5
xterm -e " source devel/setup.bash; source devel/setup.bash; rostopic pub add_marker/position geometry_msgs/Point \"x: 1\" " &
sleep 1
xterm -e " source devel/setup.bash; source devel/setup.bash; rostopic pub add_marker/state std_msgs/Int16 \"data: 1\" " &
sleep 5
xterm -e " source devel/setup.bash; rostopic pub add_marker/state std_msgs/Int16 \"data: 0\" " & 
xterm -e " source devel/setup.bash; rostopic pub add_marker/position geometry_msgs/Point \"x: -1\"; " &
sleep 5
xterm -e " source devel/setup.bash; rostopic pub add_marker/state std_msgs/Int16 \"data: 1\" " &
