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
#include <math.h>
#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/common/time.h>
#define foreach BOOST_FOREACH
//#include "hg_sdf.vs"
//#include <boost/python.hpp>
#define PI 3.14159265
using namespace glm;
typedef sensor_msgs::PointCloud2 MsgCloud;
std::vector<float> ray_cast_mean;

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
    cloud->points.erase((std::unique(cloud->points.begin(), cloud->points.end(), equalPoint)),cloud->points.end());
}

bool raycast(optimization_baxter::raycast::Request &req,
                 optimization_baxter::raycast::Response &res)
{

  ROS_INFO("in raycast service");

  float resolution = 0.01;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(req.obj_cloud, *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_save (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::VectorType voxelsInRay, voxelsInRay2;

  std::vector<int> indicesInRay, indicesInRay2;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
  octree.defineBoundingBox (0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

//  octree.deleteTree();
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  double theta = 0.0;
  double phi = 0.0;
  double R = 3.5;
  double b_secs =ros::Time::now().toSec();
  for (theta=-26.5;theta<=26.5;theta=theta+1) {
    for (phi=-26.5;phi<=26.5;phi=phi+1){
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

    octree.getIntersectedVoxelIndices(o, dir, indicesInRay);

    if (indicesInRay.size()!=0){
         int count = indicesInRay.size();
//         std::cout <<"size: " <<count <<std::endl;
         for (int i = 0; i < count;i++)
         {
               if(ray_cast_mean[indicesInRay[i]]>0){
                   continue;
               }else{
                    float ray_i = fabs(ray_cast_mean[indicesInRay[i]]);
                    float ray_i_1 = fabs(ray_cast_mean[indicesInRay[i-1]]);
                    if(ray_i<=0.02){
                       res.array.push_back(indicesInRay[i]);
                       break;
                    }else if(ray_i_1<=0.01){
                       res.array.push_back(indicesInRay[i]);
                       break;
                    }else{
                       break;
                    }
//                    if(fabs(ray_cast_mean[indicesInRay[i]])>fabs(ray_cast_mean[indicesInRay[i-1]])){
//                      res.array.push_back(indicesInRay[i-1]);
//                      break;
//                    }else{
//                      res.array.push_back(indicesInRay[i]);
//                      break;
//                    }
               }
         }

    }else{
//        ROS_INFO("no point");
    }

    }
  }
double f_secs =ros::Time::now().toSec();
  double secs =f_secs - b_secs;
  std::cout <<secs <<std::endl;
  vec3 test_point = vec3(req.p_center.y,3.5-req.p_center.x,req.p_center.z);
  float sd = fCone(test_point,1.94,3.5);
  res.distance = sd;

  res.success = true;
  ROS_INFO("finish ray casting");
  return true;
}

int main (int argc, char** argv)
{

  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::ServiceServer obj_seg_srv_;
  obj_seg_srv_ = n.advertiseService("raycast",raycast);
  ROS_INFO("ray casting service online");

  //read the mean list from rosbag
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("floatarray"));
  topics.push_back(std::string("numbers"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const m, view)
  {
        std_msgs::Float32MultiArray::ConstPtr t = m.instantiate<std_msgs::Float32MultiArray>();
        if (t != NULL)
           ray_cast_mean = t->data;

        std_msgs::Float32::ConstPtr i = m.instantiate<std_msgs::Float32>();
        if (i != NULL)
            std::cout << i->data << std::endl;
  }

  bag.close();
  std::cout <<"ray_cast_mean:" <<ray_cast_mean.size() <<std::endl;

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
