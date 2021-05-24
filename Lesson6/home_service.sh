#!/bin/sh 
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch add_markers RVIZ_Navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 15
xterm -e " source devel/setup.bash; rostopic echo /move_base/goal " &
sleep 1
xterm -e " source devel/setup.bash; rostopic echo /move_base/result " &
sleep 5
xterm -e " source devel/setup.bash; rosrun add_markers marker_shape " &
sleep 5
xterm -e " source devel/setup.bash; ./devel/lib/home_service/home_service " &
sleep 5
xterm -e " source devel/setup.bash; ./devel/lib/pick_objects/pick_objects " &
sleep 5
