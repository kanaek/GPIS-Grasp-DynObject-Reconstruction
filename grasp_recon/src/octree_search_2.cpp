#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <optimization_baxter/raycast.h>
#include <std_msgs/Int16MultiArray.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
//#include "hg_sdf.vs"
//#include <boost/python.hpp>
#define PI 3.14159265
using namespace glm;
typedef sensor_msgs::PointCloud2 MsgCloud;
bool comparePoint(pcl::PointXYZ p1, pcl::PointXYZ p2){
if (p1.x != p2.x)
    return p1.x > p2.x;
else if (p1.y != p2.y)
    return  p1.y > p2.y;
else
    return p1.z > p2.z;
}

//signed distance for cone
float sgn(float x) {
	return (x<0)?-1:1;
}

vec2 sgn(vec2 v) {
	return vec2((v.x<0)?-1:1, (v.y<0)?-1:1);
}

float fCone(vec3 p, float radius, float height) {
    vec2 dis = vec2(p.x,p.z);
	vec2 q = vec2(length(dis), p.y);
	vec2 tip = q - vec2(0, height);
	vec2 mantleDir = normalize(vec2(height, radius));
	float mantle = dot(tip, mantleDir);
	float d = max(mantle, -q.y);
	float projected = dot(tip, vec2(mantleDir.y, -mantleDir.x));

	// distance to tip
	if ((q.y > height) && (projected < 0)) {
		d = max(d, length(tip));
	}

	// distance to base ring
	if ((q.x > radius) && (projected > length(vec2(height, radius)))) {
		d = max(d, length(q - vec2(radius, 0)));
	}
	return d;
}

bool equalPoint(pcl::PointXYZ p1, pcl::PointXYZ p2){
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    return false;
}

void cullDuplePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    std::sort(cloud->points.begin(), cloud->points.end(), comparePoint);
    //auto unique_end = std::unique(cloud->points.begin(), cloud->points.end(), equalPoint);
    cloud->points.erase((std::unique(cloud->points.begin(), cloud->points.end(), equalPoint)),cloud->points.end());
}
/*
void ray_casting(MsgCloud input_cloud, MsgCloud output_cloud){
  float resolution = 0.01;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input_cloud, *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::VectorType voxelsInRay, voxelsInRay2;

  std::vector<int> indicesInRay, indicesInRay2;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
  octree.defineBoundingBox (0.0, 0.0, 0.0, 100.0, 100.0, 100.0);
  std::cout <<"set resolution done" <<std::endl;

  double theta = 0.0;
  double phi = 0.0;
  double R = 3.5;
  for (theta=0.0;theta<=90.0;theta=theta+2.8) {
    for (phi=0.0;phi<=90.0;phi=phi+1.3){
      float x = R*cos ( theta * PI / 180.0 )*cos ( phi * PI / 180.0 );
      float y = R*cos ( theta * PI / 180.0 )*sin ( phi * PI / 180.0 );
      float z = R*sin ( theta * PI / 180.0 );
    Eigen::Vector3f p(static_cast<float> (x),
                      static_cast<float> (y),
                      static_cast<float> (z));

    // origin
    Eigen::Vector3f o(static_cast<float> (0.0),
                      static_cast<float> (0.0),
                      static_cast<float> (0.0));

    geometry_msgs::Point p1, p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    p2.x = p[0];
    p2.y = p[1];
    p2.z = p[2];
    // direction vector
    Eigen::Vector3f dir(p - o);
    octree.deleteTree();
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    octree.getIntersectedVoxelCenters(o, dir, voxelsInRay,1);
      int count = voxelsInRay.size();

      for (int i = 0; i < count;i++)
      {
        cloud_save->push_back(voxelsInRay[i]);
        std::cout <<i << voxelsInRay[i] << std::endl;
      }
    //octree.getIntersectedVoxelIndices(o, dir, indicesInRay);
    }
  }
  
  cullDuplePoints(cloud_save);
  
  cloud_save->header.frame_id = "/camera_link";
  cloud_save->width    = cloud_save->points.size();
  cloud_save->height   = 1;
  cloud->header.frame_id = "/camera_link";
  cloud->width    = cloud->points.size();
  cloud->height   = 1;
  pcl::toROSMsg(*cloud, output_cloud);
  std::cout <<"finish ray casting" <<std::endl;
}

BOOST_PYTHON_MODULE(liboctree_search_2) {
    // An established convention for using boost.python.
    using namespace boost::python;

    // Expose the function hello().
    def("ray_casting", ray_casting);
}
*/
bool raycast(optimization_baxter::raycast::Request &req,
                 optimization_baxter::raycast::Response &res)
{

  ROS_INFO("in raycast service");
  //std_msgs::Int16MultiArray array;
  float resolution = 0.006;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(req.obj_cloud, *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::VectorType voxelsInRay, voxelsInRay2;

  std::vector<int> indicesInRay, indicesInRay2;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
  octree.defineBoundingBox (0.0, 0.0, 0.0, 10.0, 10.0, 10.0);
  octree.deleteTree();
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();
  double theta = 0.0;
  double phi = 0.0;
  double R = 3.5;
  for (theta=-26.5;theta<=26.5;theta=theta+0.7) {
    for (phi=-26.5;phi<=26.5;phi=phi+0.7){
      float x = R*cos ( theta * PI / 180.0 )*cos ( phi * PI / 180.0 );
      float y = R*cos ( theta * PI / 180.0 )*sin ( phi * PI / 180.0 );
      float z = R*sin ( theta * PI / 180.0 );
    Eigen::Vector3f p(static_cast<float> (x),
                      static_cast<float> (y),
                      static_cast<float> (z));

    // origin
    Eigen::Vector3f o(static_cast<float> (0.0),
                      static_cast<float> (0.0),
                      static_cast<float> (0.0));

    geometry_msgs::Point p1, p2;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    p2.x = p[0];
    p2.y = p[1];
    p2.z = p[2];
    // direction vector
    Eigen::Vector3f dir(p - o);
    /*


    octree.getIntersectedVoxelCenters(o, dir, voxelsInRay,1);
      int count = voxelsInRay.size();

      for (int i = 0; i < count;i++)
      {
        cloud_save->push_back(voxelsInRay[i]);

        //std::cout << voxelsInRay[i].x << std::endl;
      }*/
      octree.getIntersectedVoxelIndices(o, dir, indicesInRay,1);
      if (indicesInRay.size()!=0){
         //std::cout <<indicesInRay[0] << std::endl;
         res.array.push_back(indicesInRay[0]);
      }

    }
  }
  vec3 test_point = vec3(req.p_center.y,3.5-req.p_center.x,req.p_center.z);
  float sd = fCone(test_point,1.94,3.5);
  res.distance = sd;

  //cullDuplePoints(cloud_save);
  /*
  cloud_save->header.frame_id = "/camera_link";
  cloud_save->width    = cloud_save->points.size();
  cloud_save->height   = 1;
  pcl::toROSMsg(*cloud_save, res.ray_cast_cloud);*/
  res.success = true;
  ROS_INFO("finish ray casting");
  return true;
}

int main (int argc, char** argv)
{

//  fCone(vec3 p, float radius, float height)
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::ServiceServer obj_seg_srv_;
  obj_seg_srv_ = n.advertiseService("raycast",raycast);
  ROS_INFO("ray casting service online");

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE_LIST;


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
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    int count = 10;
    marker.lifetime = ros::Duration();
    geometry_msgs::Point p1, p2;
    p1.x = 3.5f;
    p1.y = 0.3f;
    p1.z = 0.0f;
    marker.points.push_back(p1);




  static tf::TransformBroadcaster br;
  tf::StampedTransform transform_rec;
  tf::TransformListener listener;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(3.5, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, PI/2);
  transform.setRotation(q);



  //publish sensor cone
  //radius=1.94,height = 3.5
  vec3 test_point = vec3(p1.y,3.5-p1.x,p1.z);
  float sd = fCone(test_point,1.94,3.5);
  std::cout <<"signed distance " <<sd <<std::endl;
  ros::Publisher pub_range = n.advertise<sensor_msgs::Range>("range", 1000);
  sensor_msgs::Range range_msg;
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  "/camera_link";
  range_msg.field_of_view = 1.0222909662;  // 58degrees
  range_msg.min_range = 0.8;
  range_msg.max_range = 3.5;
  ros::Rate r(30);
  while (ros::ok())
  {
     range_msg.range = 3.5;
     pub_range.publish(range_msg);
     marker_pub.publish(marker);
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "cone"));
     ros::spinOnce();

     r.sleep();
  }

   return 0; 
}
