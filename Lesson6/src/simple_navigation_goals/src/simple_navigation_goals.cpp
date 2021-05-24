#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  //use the command 'rostopic list | grep move_base.*/goal' to find the topic name
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal_drop_off;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal_drop_off.target_pose.header.frame_id = "map";
  goal_drop_off.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;
  goal_drop_off.target_pose.pose.position.x = 4.0;
  goal_drop_off.target_pose.pose.orientation.w = 4.0;
  
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has to travel to the desired pickup zone");
  else
  {
    ROS_INFO("The base failed to move to the desired pickup zone for some reason");
    return 0;
  }
  
  //wait 5 seconds
  ros::Duration(5.0).sleep(); // sleep for half a second
  
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(goal_drop_off);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has reached the drop off zone.");
  else
  {
    ROS_INFO("The base failed to reach the drop off zone for some reason");
    return 0;
  }
  
  return 0;
}
