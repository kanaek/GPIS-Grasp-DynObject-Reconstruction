#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <ctime>
#include <math.h>

#define PI 3.14159265
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker1", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::LINE_LIST;


    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/camera_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    int count = 10;
    marker.lifetime = ros::Duration();
    double theta = 0.0;
  double phi = 0.0;
  double R = 3.5;
  for (theta=-26.5;theta<=26.5;theta=theta+3) {
    for (phi=-26.5;phi<=26.5;phi=phi+3){
      float x = R*cos ( theta * PI / 180.0 )*cos ( phi * PI / 180.0 );
      float y = R*cos ( theta * PI / 180.0 )*sin ( phi * PI / 180.0 );
      float z = R*sin ( theta * PI / 180.0 );

    geometry_msgs::Point p1, p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    p2.x = x;
    p2.y = y;
    p2.z = z;
    marker.points.push_back(p1);
    marker.points.push_back(p2);
     }
    }
    // Publish the marker
  while (ros::ok())
  {
    marker_pub.publish(marker);

    // Cycle between different shapes
    

    r.sleep();
  }
}
