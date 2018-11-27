#!/usr/bin/env python
import rospy
import tf2_ros
import tf
#from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
#import moveit_msgs.msg
from optimization_baxter.srv import *
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import sys
import numpy as np
from rospkg import RosPack
import PyKDL
import baxter_interface
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
#joint trajectory call for baxter
from joint_trajectory import Trajectory
from geometry_msgs.msg import Pose
rp=RosPack()
rp.list()

#path=rp.get_path('ll4ma_kdl')+'/scripts'

'''
l_gripper_r_finger_joint:lower="-0.020833" upper="0.0"
l_gripper_l_finger_joint:lower="0.0" upper="0.020833"
'''
import baxter_interface
from baxter_interface import CHECK_VERSION

#sys.path.insert(0,path)
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
import  pyOpt_cost_function as cost_fn
from kdl_baxter import *
#from kdl_lbr4_model import *
#from kdl_lbr4_allegro_model import *
from kdl_baxter_hand_model import *
baxter_urdf_path=rp.get_path('baxter_description')
baxter_urdf_path = baxter_urdf_path+'/urdf/baxter2_correct_2.urdf'
# from optimization_baxter.srv import raycast
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
print baxter_urdf_path
import pyOpt
from sensor_msgs.msg import PointCloud2
from vis_robot import VizInHand
from pykdl_utils.kdl_kinematics_pointcloud import KDLKinematics_pointcloud
from urdf_parser_py.urdf import Robot
import tf2_geometry_msgs
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src/ik_planner')
from optimization_baxter.srv import raycast
from geometry_msgs.msg import Point
import roslib


rospy.init_node("grasp_client")
def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, t.transform.translation.y,
                                    t.transform.translation.z))

def transform_cloud(cloud, transform):
    t_kdl = transform_to_kdl(transform)
    points_out = []
    for p_in in cloud:
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        points_out.append((p_out[0], p_out[1], p_out[2]))
    return points_out

def call_ray_cast(cloud_out_, cloud_center, test_cloud):
    point_center = Point()
    point_center.x = cloud_center[0]
    point_center.y = cloud_center[1]
    point_center.z = cloud_center[2]
    HEADER = Header(frame_id='/camera_link')
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)
              ]
    obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_)
    rospy.wait_for_service('raycast')
    raycast_srv = rospy.ServiceProxy('raycast', raycast)
    raycast_resp = raycast_srv(obj_cloud_gpis, point_center)
    temp_list = raycast_resp.array
    distance = raycast_resp.distance
    after_ray = []
    for i in range(len(temp_list)):
        after_ray.append(list(cloud_out_[temp_list[i]][0:3]))

    after_ray = np.array(after_ray)
    return after_ray,distance,temp_list


tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)
camera_link = tf_buffer.lookup_transform('camera_link',
                                         'camera_rgb_optical_frame',  # source frame
                                         rospy.Time(0),  # get the tf at first available time
                                         rospy.Duration(1.0))


test_cloud = np.load('new_xx_all.npy')
test_cloud = np.asarray(test_cloud,dtype=float)
cloud_out = transform_cloud(test_cloud, camera_link)

cloud_center = np.sum(cloud_out,axis=0)
cloud_center = np.dot(cloud_center, 1.0 / len(cloud_out))
print('cloud_center',cloud_center)
tf_send = []
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "camera_link"
t.child_frame_id = "cloud_center"
t.transform.translation.x = cloud_center[0]
t.transform.translation.y = cloud_center[1]
t.transform.translation.z = cloud_center[2]
t.transform.rotation.x = 0
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 1
tf_send.append(t)
br = tf2_ros.StaticTransformBroadcaster()
br.sendTransform(tf_send)


after_ray, distance,temp_list = call_ray_cast(cloud_out,cloud_center,test_cloud)

if after_ray.shape[0] == 0:
    print('out of scope')
else:
    print('in scope',after_ray)

HEADER = Header(frame_id='/camera_link')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
obj_cloud_center = pc2.create_cloud(HEADER, fields, after_ray)
cloud_pub = rospy.Publisher('gpis_debug_center',PointCloud2,queue_size=10)


new_mean = np.load('new_mean_all.npy')
print('test_cloud',len(test_cloud))
print('new_mean',len(new_mean))
mean_after_ray = [new_mean[temp_list[j]] for j in range(0, len(temp_list))]
print('mean_after_ray',mean_after_ray)
mean_for_ros_bag = list(new_mean)
'''
//////////////////////////////////////////////////////////////////////////////////
('mean_after_ray', [0.16511971755880112, 0.1596803563974325, 0.15261676624543996, 
0.1438931519519033, 0.13355390038183851, 0.12173363095936954, 0.10866020639768004,
 0.094649573280640942, 0.080092278038821602, 0.065432604187512261, 0.05114232923665249, 
 0.03769192294396595, 0.025522446330252191, 0.0150213649128903, 0.0065049434885603441, 
 0.00020891746508538978, -0.0037121077970356242, -0.0051773833729987784, -0.00416686421255269, 
 -0.00070495313666235626, -0.0069899201133633015, -0.0017040539768822249, 0.0060990424647895392, 
 0.016397776753760918, 0.029159770386774364, 0.044334211387706768, 0.061840831272510999, 0.081557885945228059, 
 0.10331173001939431, 0.126870324476416])
 
('cloud_center', array([ 1.11938321, -0.05914543, -0.01687851]))
('in scope', array([[ 1.11999995, -0.05674914, -0.01390436]]))
('test_cloud', 4527)
('new_mean', 4527)
('mean_after_ray', [0.00020891746508538978])
before debug

'''
# import rosbag
# from std_msgs.msg import Float32,Float32MultiArray
#
# bag = rosbag.Bag('test.bag', 'w')
#
# try:
#
#     i = Float32()
#     i.data = 42
#
#     x = Float32MultiArray()
#     for j in range(len(mean_for_ros_bag)):
#         x.data.append(mean_for_ros_bag[j])
#
#     bag.write('numbers', i)
#     bag.write('floatarray', x)
# finally:
#     bag.close()


'''
////////////////////////////////////////////////////////////
plot all points intersected by a ray
'''
import matplotlib.pyplot as plt

t = np.arange(0., len(mean_after_ray), 1)
print('len',t,len(mean_after_ray))
numpy_array = np.array(mean_after_ray)
plt.plot(t, mean_after_ray, 'r--')
plt.show()



HEADER = Header(frame_id='/camera_link')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out)
cloud_pub_after1 = rospy.Publisher('gpis_debug',PointCloud2,queue_size=10)
rate = rospy.Rate(10)
print('before debug')
while not rospy.is_shutdown():
    cloud_pub_after1.publish(obj_cloud_gpis)
    cloud_pub.publish(obj_cloud_center)
    rate.sleep()


