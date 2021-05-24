#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>

#define GOAL_POSITION_TOPIC		"/move_base/goal"
#define ACTION_STATUS_TOPIC		"/move_base/result"

typedef struct __MarkerStatus
{
	char * current_goal_id[1024];
    double current_goal_x;
    double current_goal_y;
    double current_goal_z;
    int current_goal_status;	//-1: waiting, 0: goal received, 1: goal reached
} MarkerStatus;

static visualization_msgs::Marker gMarker;
static ros::Publisher marker_pub;
static MarkerStatus gState;

void update_marker_position(const geometry_msgs::Point::ConstPtr& input)
{
    ROS_INFO("Received message 'add_marker_position'");
  
    gMarker.pose.position.x = input->x;
    gMarker.pose.position.y = input->y;
    gMarker.pose.position.z = input->z;
}

void update_marker_state(const std_msgs::Int16::ConstPtr& input)
{  
   int m_choice = input->data;
   ROS_INFO("Received message 'add_marker_state' choice made %d", m_choice);
  
   switch( m_choice )
   {
     case 0:	//disable marker visibility
       gMarker.action = visualization_msgs::Marker::DELETE;
       marker_pub.publish(gMarker);       
       ROS_INFO("disable the marker");
       break;
     case 1:	//enable marker visibility
       gMarker.color.r = 0.0f;
       gMarker.color.g = 1.0f;
       gMarker.color.b = 0.0f;
       gMarker.color.a = 1.0;
       gMarker.action = visualization_msgs::Marker::ADD;
       marker_pub.publish(gMarker);
       ROS_INFO("enable marker position x:%f, y:%f, z:%f", gMarker.pose.position.x, gMarker.pose.position.y, gMarker.pose.position.z);
       break;
     case -1:	//invalid marker
       gMarker.action = visualization_msgs::Marker::DELETE;
       marker_pub.publish(gMarker);
       gMarker.color.r = 1.0f;
       gMarker.color.g = 0.0f;
       gMarker.color.b = 0.0f;
       gMarker.color.a = 1.0;       
       gMarker.action = visualization_msgs::Marker::ADD;
       marker_pub.publish(gMarker);
       ROS_INFO("not reachable marker position x:%f, y:%f, z:%f mark red", gMarker.pose.position.x, gMarker.pose.position.y, gMarker.pose.position.z);
       break;
   }
}

/*
void rostopic_move_base_status(const move_base_msgs::MoveBaseActionResult& input)
{  
   char msg[1024];
   int m_status = input.status.status;
  
   //sprintf(msg,"Received message '/move_base/result' status %d",input.status);
   ROS_INFO("Received message '/move_base/result' status %x",m_status);
  
   switch( m_status )
   {
     case 0://PENDING       
       gState.current_goal_status = -1;
       break;
     case 1://ACTIVE
       gState.current_goal_status = 0;
       break;
     case 3://SUCCEEDED
       gState.current_goal_status = 1;
       break;
     case 2://PREEMPTIVE
     case 4://ABORTED
     case 5://REJECTED
     case 6://PREEMPTIVE
     case 7://RECALLING
     case 8://RECALLED
     case 9://LOST
       break;
   }
}
*/

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  ros::Subscriber marker_position = n.subscribe<geometry_msgs::Point>("add_marker/position",1,&update_marker_position);
  ros::Subscriber marker_state = n.subscribe<std_msgs::Int16>("add_marker/state",1,&update_marker_state);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  gState.current_goal_status = -1;
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  gMarker.header.frame_id = "map";
  gMarker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  gMarker.ns = "basic_shapes";
  gMarker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  gMarker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  gMarker.pose.position.x = 0;
  gMarker.pose.position.y = 0;
  gMarker.pose.position.z = 0;
  gMarker.pose.orientation.x = 0.0;
  gMarker.pose.orientation.y = 0.0;
  gMarker.pose.orientation.z = 0.0;
  gMarker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  gMarker.scale.x = 0.5;
  gMarker.scale.y = 0.5;
  gMarker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  gMarker.color.r = 0.0f;
  gMarker.color.g = 1.0f;
  gMarker.color.b = 0.0f;
  gMarker.color.a = 1.0;

  gMarker.lifetime = ros::Duration();

  ros::spin();
}

/*
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber marker_position = n.subscribe<geometry_msgs::Point>("/add_marker/position",1,&update_marker_position);
  ros::Subscriber marker_position = n.subscribe<int>("/add_marker/state",1,&update_marker_state);
  
  //-- commented out, require to call 'ros::spin();'
  //ros::Subscriber marker_sub_goal = n.subscribe("/move_base/goal", 1, &rostopic_move_base_goal);
  //ros::Subscriber marker_sub_status = n.subscribe("/move_base/result", 1, &rostopic_move_base_status);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  gState.current_goal_status = -1;
  
  //while (ros::ok())
  //{
    //visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    gMarker.header.frame_id = "map";
    gMarker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    gMarker.ns = "basic_shapes";
    gMarker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    gMarker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    gMarker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    gMarker.pose.position.x = 0;
    gMarker.pose.position.y = 0;
    gMarker.pose.position.z = 0;
    gMarker.pose.orientation.x = 0.0;
    gMarker.pose.orientation.y = 0.0;
    gMarker.pose.orientation.z = 0.0;
    gMarker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    gMarker.scale.x = 0.5;
    gMarker.scale.y = 0.5;
    gMarker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    gMarker.color.r = 0.0f;
    gMarker.color.g = 1.0f;
    gMarker.color.b = 0.0f;
    gMarker.color.a = 1.0;

    gMarker.lifetime = ros::Duration();

    //STEP 1: Initially show the marker at the pickup zone
    /////////////////////////////////////////////////////////////////////////
    //wait for the subscribers from RVIZ launched with marker (for monitoring)
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
  
    // Wait for the first goal to publish
    // subscript to '/move_base/goal'   (rostopic type /move_base/goal | rosmsg show)
    boost::shared_ptr<move_base_msgs::MoveBaseActionGoal const> gFirstGoalInfo;
    ROS_INFO("Wait for '/move_base/goal' to publish the first goal");
    gFirstGoalInfo = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", ros::Duration(30));
    if(gFirstGoalInfo != NULL){
       rostopic_update_goal_position(*gFirstGoalInfo);
    }
    // read and updated the position information
    //          float64 goal.target_pos.pos.position.x
    //          float64 goal.target_pos.pos.position.y
    gMarker.pose.position.x = gState.current_goal_x;
    gMarker.pose.position.y = gState.current_goal_y;
    gMarker.pose.position.z = gState.current_goal_z;
    gState.current_goal_status = 0;
  
    // Publish the marker
    ROS_INFO("pickup marker - published waiting for robot to reach the pickup node");
    marker_pub.publish(gMarker);

    //STEP 2: Hide the marker once your robot reaches the pickup zone
    /////////////////////////////////////////////////////////////////////////
    //subscript to '/move_type/status'
    //read and wait until the status = 3   (rostopic echo /move_type/status)
    //          uint8 status_list.status == 3 (goal reached)
    int iReceived = 0;
    while( iReceived==0 )
    {
       boost::shared_ptr<move_base_msgs::MoveBaseActionResult const> gFirstGoalReached;
       gFirstGoalReached = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionResult>("/move_base/result", ros::Duration(30));
       if(gFirstGoalReached != NULL){
          rostopic_move_base_status(*gFirstGoalReached);
          iReceived = 1;
       }
       else
       {
          ROS_WARN("no message has received");
       }
    }
    ROS_INFO("pickup marker - reached, remove marker");
    gState.current_goal_status = -1;
    //delete the marker
    gMarker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(gMarker);
  
    //STEP 3: Wait 5 seconds to simulate a pickup
    /////////////////////////////////////////////////////////////////////////
    boost::shared_ptr<move_base_msgs::MoveBaseActionGoal const> gSecondGoalInfo;
    ROS_INFO("Wait for '/move_base/goal' to publish the second goal");
    gSecondGoalInfo = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", ros::Duration(30));
    if(gSecondGoalInfo != NULL){
       rostopic_update_goal_position(*gSecondGoalInfo);
    }
    // read and updated the position information
    //          float64 goal.target_pos.pos.position.x
    //          float64 goal.target_pos.pos.position.y
    gMarker.pose.position.x = gState.current_goal_x;
    gMarker.pose.position.y = gState.current_goal_y;
    gMarker.pose.position.z = gState.current_goal_z;
    gState.current_goal_status = 0;
  
    //STEP 4: Show the marker at the drop off zone once your robot reaches it
    /////////////////////////////////////////////////////////////////////////
    //subscript to '/move_type/status'
    //read and wait until the status = 3   (rostopic echo /move_type/status)
    //          uint8 status_list.status == 3 (goal reached)
    boost::shared_ptr<move_base_msgs::MoveBaseActionResult const> gSecondGoalReached;
    while( gState.current_goal_status!=1 )
    {
       ROS_WARN("drop off marker - received, waiting for robot to reach the drop off node");
       gSecondGoalReached = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionResult>("/move_base/result", ros::Duration(30));
       if(gSecondGoalReached != NULL){
          rostopic_move_base_status(*gSecondGoalReached);
       }
    }
    gState.current_goal_status = -1;
    // Publish the marker
    ROS_INFO("Publish drop off marker - reached, show marker");
    gMarker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(gMarker);
  
  sleep(5);
}
*/