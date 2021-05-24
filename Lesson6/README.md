## Table of contents
* [General info](#general-info)
* [Project Build](#build)
* [Project Execution](#execution)

## General info
In this project is created using the steps from Udacity Lesson 6.  The result of this project is submit to Udactiy as the project 7.

This project includes 9 sub-projects which 6 of them are sourced from github and 3 are created for this project.  The following described the sub-projects.

1. add_markers
    project is created using 'catkin_create_pkg add_markers roscpp visualization_msgs'
    this project created a node that:
        - adds two topics that other nodes can subscribe: add_marker/position, add_marker/state
        - send marker visual information to RVIZ by publish
    contains a customized RVIZ environment launcher file includes marker visualization 
2. home_service
    project is created using 'catkin_create_pkg home_service roscpp move_base_msgs'
    this project created a node that:
        - monitor the goal and robot status by subscribe to /move_base/goal, /move_base/result
        - controls the markers to 'add_marker' node by publish
3. pick_objects
    project is created using 'catkin_create_pkg pick_objects move_base_msgs actionlib roscpp'
    this project is implemented using 'actionlib::SimpleActionClient' to created a node that:
        - sets a pickup goal
        - wait for the action is done
        - sets a drop off goal
4. simple_navigation_goals
    Example project created using example from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
    Not an active project.  Retained in this project folder for example purpose.  
5. slam_gmapping
    Public ROS package copied from github
    Reference: http://wiki.ros.org/slam_gmapping?distro=noetic
6. turtlebot
    Public ROS package copied from github
    Reference: http://wiki.ros.org/turtlebot?distro=kinetic
7. turtlebot_interactions
    Public ROS package copied from github
    Reference: http://wiki.ros.org/turtlebot_interactions?distro=kinetic
8. turtlebot_simulator
    Public ROS package copied from github
    Reference: http://wiki.ros.org/turtlebot_simulator?distro=kinetic
9. using_markers
    Example project created using example from http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
    Not an active project.  Retained in this project folder for example purpose.  

## build
Project requires:
* Udacity Workspace or a Virtual Environment with image provided by Udacity
* Gazebo
* RVIZ

1. Log into the the Udacity Workspace
2. Open a console, download and build the project
```
$ cd /home/workspace
$ git clone https://github.com/peggycyggep/Lesson6
$ cd Lesson6
$ catkin_make
```

## execution
Per project requirement, the following script files are created

1. test_slam.sh
script file to used to check out the SLAM setting.  After the execution, manually control of the robot using teleop is expected to generate map.

2. test_navigation.sh
script file to setup the navigation environment

3. pick_objects.sh
script file to send two goals for the robot to reach, after 'test_navigation.sh' is executed.

4. add_marker.sh
script file to test out the add_marker node.  The execution of this script file will launch the RVIZ with marker visualize enabled.  Terminal publishes simulated marker position and states to simulate the scenario:The marker should initially be published at the pickup zone. After 5 seconds it should be hidden. Then after another 5 seconds it should appear at the drop off zone.

5. home_service.sh
script file to launch the environment, the pick_objects node, add_markers node, and home_service node.  User can see in the RVIZ that marker is first enabled as green square box at the pickup location.  After robot reached the pickup location, marker is disappeared.  As robot reaches the drop off location, marker will show up again as green square box.  

During the simulation, it also happened if robot failed to reach drop off location.  In this scenario, the marker will show up as red square box.



