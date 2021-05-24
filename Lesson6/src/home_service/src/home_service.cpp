#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>

#define GOAL_POSITION_TOPIC		"/move_base/goal"
#define ACTION_STATUS_TOPIC		"/move_base/result"
#define ACTION_STATUS_MSG_TYPE	move_base_msgs::MoveBaseActionResult
#define GOAL_POSITION_MSG_TYPE	move_base_msgs::MoveBaseActionGoal

typedef struct __MarkerStatus
{
    geometry_msgs::Point current_pos;
    std_msgs::Int16	current_marker_state;
	char * current_goal_id[1024];
    int current_goal_status;	//-1: waiting, 0: goal received, 1: goal reached
} MarkerStatus;

static ros::Publisher send_position;
static ros::Publisher send_state;
static MarkerStatus gState;

void update_marker_position(const move_base_msgs::MoveBaseActionGoal::ConstPtr& input)
{
   ROS_INFO("Received message '/move_base/goal' goal posx %lf posy %lf",input->goal.target_pose.pose.position.x,input->goal.target_pose.pose.position.y);
  
   gState.current_pos.x = input->goal.target_pose.pose.position.x;//input.goal.target_pose.position.x;
   gState.current_pos.y = input->goal.target_pose.pose.position.y;//input.goal.target_pose.position.y;
   gState.current_pos.z = 0;
   
  //update the marker position
  send_position.publish(gState.current_pos);
  
   if( gState.current_goal_status==-1 )
   {
      gState.current_marker_state.data = 1;
      send_state.publish(gState.current_marker_state);
      gState.current_goal_status = 0;
   }
  
   //marker_pub = n.advertise<std_msgs::Int16>("add_marker/state", 1);  
}

void update_marker_state(const move_base_msgs::MoveBaseActionResult::ConstPtr& input)
{  
   int m_status = input->status.status;
  
   ROS_INFO("Received message '/move_base/result' status %x",m_status);
  
   /*switch( m_status )
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
   }*/
  
   if( gState.current_goal_status==0 )
   {
         if( m_status==3 )
         {
         	gState.current_marker_state.data = 0;
         }
         else
         {
         	gState.current_marker_state.data = -1;
         }
         
         send_state.publish(gState.current_marker_state);
         gState.current_goal_status = 1;
   }
   else if( gState.current_goal_status==1 )
   {
         if( m_status==3 )
         {
         	gState.current_marker_state.data = 1;
         }
         else
         {
         	gState.current_marker_state.data = -1;
         }
         send_state.publish(gState.current_marker_state);
         gState.current_goal_status = -1;
   }  
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "home_service");
  ros::NodeHandle n;  
  ros::Rate r(1);
  send_position = n.advertise<geometry_msgs::Point>("add_marker/position", 1);
  send_state = n.advertise<std_msgs::Int16>("add_marker/state", 1);
  
  gState.current_goal_status = -1;
  ros::Subscriber marker_position = n.subscribe<GOAL_POSITION_MSG_TYPE>(GOAL_POSITION_TOPIC,1,&update_marker_position);
  ros::Subscriber marker_state = n.subscribe<ACTION_STATUS_MSG_TYPE>(ACTION_STATUS_TOPIC,1,&update_marker_state);

  ROS_INFO("home_service ready, waiting for message");
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
