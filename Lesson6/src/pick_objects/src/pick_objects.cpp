#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal goal_drop_off;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal_drop_off.target_pose.header.frame_id = "map";
  goal_drop_off.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;
  goal_drop_off.target_pose.pose.position.x = -1.0;
  goal_drop_off.target_pose.pose.orientation.w = -1.0;
  
  ROS_INFO("Sending pick up goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has moved to the pick up location");
  else
  {
    ROS_INFO("The base failed to move to the pick up location");
    return 0;
  }

  //wait 5 seconds
  ros::Duration(5.0).sleep();
  
  ROS_INFO("Sending drop off goal");
  ac.sendGoal(goal_drop_off);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has moved to the drop off location");
  else
  {
    ROS_INFO("The base failed to move to the drop off location");
    return 0;
  }
  
  return 0;
}