#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int set_goal( MoveBaseClient &ac, float x, float y, float w )
{
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, robot moved successfully to position (%f, %f)", x, y);
    return 0;
  } else {
    ROS_INFO("The base failed to move to position (%f, %f) for some reason", x, y);
    return -1;
  }
}

void set_pose_estimate( ros::Publisher &pose_pub )
{
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.pose.pose.position.x = 0.046838644892;
  msg.pose.pose.position.y = 0.159260526299;
  msg.pose.pose.position.z = 0;
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = -0.713794180781;
  msg.pose.pose.orientation.w = 0.700355529344;
  msg.pose.covariance[0] = 0.25;
  msg.pose.covariance[7] = 0.25;
  msg.pose.covariance[35] = 0.06853891945200942;
  pose_pub.publish(msg);
}

int main( int argc, char** argv )
{
  // Initialize the pick_objects node
  ros::init( argc, argv, "pick_objects" );

  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 20);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac( "move_base", true );

  // Wait 5 sec for move_base action server to come up
  while( !ac.waitForServer( ros::Duration(5.0) ) ) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Set initial pose
  set_pose_estimate( pose_pub );

  ros::Duration(1.0).sleep();

  if( set_goal(ac, 0.0, -2.667, 1.0) ) {
    return -1;
  }

  ros::Duration(5.0).sleep();

  if( set_goal(ac, -4.5, 7.9, 1.819) ) {
    return -1;
  }
}

