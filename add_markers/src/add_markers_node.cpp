#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <add_markers/MarkerCommand.h>

static ros::Publisher marker_pub;

// handle_drive_request: a callback function that executes whenever a drive_bot service is requested
// This function publishes to the visualization_marker topic, handled by RViz
void handle_add_marker_command(const add_markers::MarkerCommandPtr &cmd)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set our Shape type to be a cube
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = cmd->pose_x;
    marker.pose.position.y = cmd->pose_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = cmd->orientation_z;
    marker.pose.orientation.w = cmd->orientation_w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.05f;
    marker.color.g = 0.3f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while( marker_pub.getNumSubscribers() < 1 ) {
      if( !ros::ok() ) {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO("Publishing marker");
    marker_pub.publish( marker );
}

void handle_delete_marker_command(const add_markers::MarkerCommandPtr &cmd)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set our Shape type to be a cube
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETEALL;

    marker_pub.publish( marker );
}

// handle_drive_request: a callback function that executes whenever a drive_bot service is requested
// This function publishes to the visualization_marker topic, handled by RViz
void handle_marker_command(const add_markers::MarkerCommandPtr &cmd)
{
    if( cmd->command == "add" ) {
        handle_add_marker_command( cmd );
    } else if( cmd->command == "delete" ) {
        handle_delete_marker_command( cmd );
    } else {
        ROS_WARN("Invalid marker command %s", cmd->command.c_str() );
    }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
  ros::Subscriber sub = n.subscribe("/add_markers/marker_command", 1, handle_marker_command);

  ros::spin();

  return 0;
}

