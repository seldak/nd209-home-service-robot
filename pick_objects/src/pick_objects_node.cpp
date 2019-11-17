#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <add_markers/MarkerCommand.h>

struct goal_pose {
    float pose_x;
    float pose_y;
    float orientation_z;
    float orientation_w;
};

enum goal_id {
    PICKUP = 0,
    DROPOFF,
    NUM_GOALS,
};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int set_goal( MoveBaseClient &ac, struct goal_pose &pose )
{
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pose.pose_x;
  goal.target_pose.pose.position.y = pose.pose_y;
  goal.target_pose.pose.orientation.z = pose.orientation_z;
  goal.target_pose.pose.orientation.w = pose.orientation_w;
  ROS_INFO("orientation = %f, %f, %f, %f",
    goal.target_pose.pose.orientation.x,
    goal.target_pose.pose.orientation.y,
    goal.target_pose.pose.orientation.z,
    goal.target_pose.pose.orientation.w);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot moved successfully to position (%f, %f)", pose.pose_x, pose.pose_y);
    return 0;
  } else {
    ROS_INFO("The base failed to move to position (%f, %f) for some reason", pose.pose_x, pose.pose_y);
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

static void add_marker( ros::Publisher &marker_pub, struct goal_pose &pose )
{
  add_markers::MarkerCommand cmd;

  cmd.command = "add";
  cmd.pose_x = pose.pose_x;
  cmd.pose_y = pose.pose_y;
  cmd.orientation_z = pose.orientation_z;
  cmd.orientation_w = pose.orientation_w;

  marker_pub.publish(cmd);
}

static void delete_marker( ros::Publisher &marker_pub )
{
  add_markers::MarkerCommand cmd;

  cmd.command = "delete";

  marker_pub.publish(cmd);
}

struct goal_pose goal_poses[NUM_GOALS] = {
  { -2.48608756065, 0.0, -0.711587503742, 0.702597483996 },
  {  7.9,           4.5, 0.0057830503341, 0.999983278025 },
};

int main( int argc, char** argv )
{
  // Initialize the pick_objects node
  ros::init( argc, argv, "pick_objects" );

  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 20);
  ros::Publisher marker_pub = n.advertise<add_markers::MarkerCommand>("/add_markers/marker_command", 5);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac( "move_base", true );

  // Wait 5 sec for move_base action server to come up
  while( !ac.waitForServer( ros::Duration(5.0) ) ) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Set initial pose
  set_pose_estimate( pose_pub );
  ros::Duration(1.0).sleep();

  struct goal_pose pickup = goal_poses[PICKUP];
  struct goal_pose dropoff = goal_poses[DROPOFF];

  add_marker( marker_pub, pickup );
  
  if( set_goal( ac, pickup ) ) {
    return -1;
  }
  delete_marker( marker_pub );

  ros::Duration(5.0).sleep();

  if( set_goal( ac, dropoff ) ) {
    return -1;
  }
  add_marker( marker_pub, dropoff );
}

