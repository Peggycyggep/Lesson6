ROS Node

http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

In order to create a ROS node that sends goals to the navigation stack, the first thing we'll need to do is create a package. To do this we'll use the handy command where we want to create the package directory with a dependency on the move_base_msgs, actionlib, and roscpp packages as shown below:

$ catkin_create_pkg simple_navigation_goals move_base_msgs actionlib roscpp

After this is done we'll need to roscd to the package we created, since we'll be using it as our workspace

roscd simple_navigation_goals
