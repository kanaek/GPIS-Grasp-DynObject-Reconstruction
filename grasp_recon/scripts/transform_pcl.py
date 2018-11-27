#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
#import moveit_commander
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
import GPflow
from geometry_msgs.msg import Point

rospy.init_node("grasp_client")
path = rp.get_path('baxter_description')
path = path + '/urdf/baxter2_correct_2.urdf'

urdf_file = file(path, 'r')
robot = Robot.from_xml_string(urdf_file.read())
chain = KDLKinematics_pointcloud(robot, 'base', 'left_gripper', 'left_gripper')


def call_ray_cast(cloud_out_, cloud_center, test_cloud,return_index=False):
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
    if return_index == False:
        after_ray = []
        for i in range(len(temp_list)):
            after_ray.append(list(test_cloud[temp_list[i]][0:3]))

        after_ray = np.array(after_ray)
        return after_ray, distance
    else:
        after_ray = []
        for i in range(len(temp_list)):
            after_ray.append(list(test_cloud[temp_list[i]][0:3]))

        after_ray = np.array(after_ray)
        return after_ray, temp_list, distance


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


tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform_optical_frame_gripper = tf_buffer.lookup_transform('left_gripper',
                                                                  'camera_rgb_optical_frame',  # source frame
                                                                  rospy.Time(0),  # get the tf at first available time
                                                                  rospy.Duration(1.0))  # wait for 1 second
transform_base_gripper = tf_buffer.lookup_transform('base',
                                                     'camera_rgb_optical_frame',  # source frame
                                                     rospy.Time(0),  # get the tf at first available time
                                                     rospy.Duration(1.0))  # wait for 1 second

base_camera = tf_buffer.lookup_transform('camera_link',
                                         'base',  # source frame
                                         rospy.Time(0),  # get the tf at first available time
                                         rospy.Duration(1.0))



test_cloud = np.load('new_xx.npy')
X = np.load('X.npy')
Y = np.load('Y.npy')
width = np.load('width.npy')
height = np.load('height.npy')
depth = np.load('depth.npy')

'''
///////////////////////////////
init gpflow
'''
k = GPflow.kernels.RBF(input_dim=3, lengthscales=0.07)
meanf = GPflow.mean_functions.Ellipsoidal(width, height, depth)
gpflow = GPflow.gpr.GPR(X, Y, kern=k, mean_function=meanf)
gpflow.likelihood.variance = 0.1

'''
///////////////////////////////
add all the points to the end effector
'''
cloud_out = transform_cloud(test_cloud, transform_optical_frame_gripper)
for i in range(len(cloud_out)):
    if i == 0:
        child_name_cloud = str(i)
        pose = cloud_out[i]
        # cloud_center = np.array(list(pose))
        chain.add_new_points_to_tree(pose, child_name_cloud)
    else:
        child_name_cloud = str(i)
        pose = cloud_out[i]
        chain.add_new_points_to_tree(pose, child_name_cloud)




cloud_out_2 = transform_cloud(test_cloud, transform_base_gripper)
num_points = len(cloud_out)
cloud_center = np.sum(cloud_out,axis=0)
cloud_center = np.dot(cloud_center, 1.0 / len(cloud_out))
cloud_center = tuple(cloud_center)
chain.add_new_points_to_tree(cloud_center, 'cloud_center')

cloud_center_2 = np.sum(cloud_out_2,axis=0)
cloud_center_2 = np.dot(cloud_center_2, 1.0 / len(cloud_out))

# rospy.sleep(20)
'''
specify the rotate angle
'''
trans_x = 0.0
trans_y = 0.0
trans_z = 0.0
rotate_angle_x = 0.0
rotate_angle_y = 0.0
rotate_angle_z = 1.57
'''
get current joint angle
'''
rs = baxter_interface.RobotEnable(CHECK_VERSION)
rs.enable()
limb_interface = baxter_interface.limb.Limb('left')
current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
c = chain.forward_pointcloud(current_angles, end_link='cloud_center', base_link='base')
print('pose',c)
pose=np.zeros(7)
R=PyKDL.Rotation(c[0,0],c[0,1],c[0,2],c[1,0],c[1,1],c[1,2],c[2,0],c[2,1],c[2,2])
pose[3:7]=R.GetQuaternion()
pose[0:3]=c[0:3,3].ravel()
center_base = pose[0:3]
print('center_base',center_base)
cloud_out_2 = cloud_out_2 - np.array(center_base)



tf_send = []
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base"
t.child_frame_id = "cloud_center"
t.transform.translation.x = pose[0]
t.transform.translation.y = pose[1]
t.transform.translation.z = pose[2]
t.transform.rotation.x = pose[3]
t.transform.rotation.y = pose[4]
t.transform.rotation.z = pose[5]
t.transform.rotation.w = pose[6]
tf_send.append(t)
br = tf2_ros.StaticTransformBroadcaster()
#
print('old_center',cloud_center)

R = tf.transformations.euler_matrix(rotate_angle_x,rotate_angle_y ,rotate_angle_z, 'sxyz')#rotate with respect to z axis

print('R',R)
new = np.dot(c,R)
print('new',new)
pose=np.zeros(7)
rpy = np.zeros(3)
R_1=PyKDL.Rotation(new[0,0],new[0,1],new[0,2],new[1,0],new[1,1],new[1,2],new[2,0],new[2,1],new[2,2])
pose[3:7]=R_1.GetQuaternion()
pose[0:3]=new[0:3,3].ravel()
# rpy = R.GetRPY()
# rpy = list(rpy)
# print('rpy',rpy)
# rpy[0] = rpy[0]+1.57
# print('rpy',rpy)
# quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], axes='rxyz')
# print('quat',quat)
# #
# #
# t = geometry_msgs.msg.TransformStamped()
# t.header.stamp = rospy.Time.now()
# t.header.frame_id = "base"
# t.child_frame_id = "cloud_center_new"
# t.transform.translation.x = pose[0]
# t.transform.translation.y = pose[1]
# t.transform.translation.z = pose[2]
# t.transform.rotation.x = quat[0]
# t.transform.rotation.y = quat[1]
# t.transform.rotation.z = quat[2]
# t.transform.rotation.w = quat[3]
# tf_send.append(t)
# br.sendTransform(tf_send)
#
# rate = rospy.Rate(10)
# print('before debug')
# while not rospy.is_shutdown():
#     rate.sleep()
'''
////////////////////////////////////////////////////////////
get new ik solution for new end effector pose
'''

# l_pose = Pose()
# l_pose.position.x = pose[0]
# l_pose.position.y = pose[1]
# l_pose.position.z = pose[2]
# l_pose.orientation.x = pose[3]
# l_pose.orientation.y = pose[4]
# l_pose.orientation.z = pose[5]
# l_pose.orientation.w = pose[6]
# from ik_plan import ik_solver
# ik_solver = ik_solver()
# from sensor_msgs.msg import JointState
# joint_angle = JointState()
# angles = baxter_interface.Limb('left').joint_angles()
# j = 0
# for i in angles:
#     joint_angle.name.append(i)
#     joint_angle.position.append(current_angles[j])
#     j += 1
# temp_ik = ik_solver.ik_solve(l_pose, joint_angle)
# print('temp_ik',temp_ik)
# jacobian = chain.jacobian(temp_ik)
# gradient = np.ones(6)
# result = np.dot(gradient,jacobian)
# result = np.asarray(result)
# result = result[0]

# temp_ik = [temp_ik]
# path = '/home/kanrun/catkin_ws/src/baxter_common/baxter_description/'
#
# urdf_path = path + '/urdf/baxter2_correct_2.urdf'
# v_hand = VizInHand(path, urdf_path)
# for i in range(0, len(temp_ik)):
#     print
#     'ii', i
#     pose_arr = []
#     temp_u = temp_ik[i]
#     for j in range(7):
#         current = temp_u[0:j + 1]
#         temp = v_hand.FK_joint(current, j)
#         pose_arr.append(temp)
# v_hand.viz_robot_traj(pose_arr)
'''
//////////////////////////////////////////////////////////////////
transform the pointcloud to the cloud_center frame
'''

rot = c[0:3,0:3]
print('rot',rot)
new_cloud = []
for i in range(len(cloud_out_2)):
    trans = np.transpose(cloud_out_2[i])
    temp_rot_cloud = np.dot(rot, trans)
    temp_rot_cloud = np.asarray(temp_rot_cloud)
    new_cloud.append(temp_rot_cloud[0])
new_cloud = np.array(new_cloud)

new_cloud_2 = []
for q in range(len(new_cloud)):
    temp = np.append(new_cloud[q],test_cloud[q,3])
    new_cloud_2.append(temp)
'''
/////////////////////////////////////////////////////////////////////////
rotate the pointcloud around its center
'''

rotate_cloud = []
R = tf.transformations.euler_matrix(rotate_angle_x, rotate_angle_y, rotate_angle_z, 'sxyz') #rotate with respect to z axis
rot = R[0:3,0:3]
# rot = np.array(rot)
new_cloud = list(new_cloud)
print('new_rot',rot)
for i in range(len(new_cloud)):
    temp_rot_cloud = np.dot(rot, new_cloud[i])
    temp_rot_cloud = np.asarray(temp_rot_cloud)
    rotate_cloud.append(temp_rot_cloud)
rotate_cloud = np.array(rotate_cloud)


new_cloud_3 = []
for q in range(len(rotate_cloud)):
    temp = np.append(rotate_cloud[q],test_cloud[q,3])
    new_cloud_3.append(temp)

'''
///////////////////////////////////////////////////////////////////////////////
'''

HEADER = Header(frame_id='/cloud_center')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 16, PointField.FLOAT32, 1)
          ]
obj_cloud_gpis = pc2.create_cloud(HEADER,fields, new_cloud_2)
cloud_pub = rospy.Publisher('gpis_debug_before',PointCloud2,queue_size=10)

'''
////////////////// transform back to base frame
'''
rotate_cloud_base = rotate_cloud
rot = c[0:3,0:3]
rot_inv = np.linalg.inv(rot)
base_cloud = []
for i in range(len(rotate_cloud_base)):
    temp_rot_cloud = np.dot(rot_inv, rotate_cloud_base[i])
    temp_rot_cloud = np.asarray(temp_rot_cloud)
    base_cloud.append(temp_rot_cloud[0])
base_cloud = np.array(base_cloud)
base_cloud = base_cloud + np.array(center_base)
# print('base_cloud',base_cloud)
new_cloud_4 = []
for q in range(len(base_cloud)):
    temp = np.append(base_cloud[q],test_cloud[q,3])
    new_cloud_4.append(temp)

'''
////////////////////////////////////////////////////////////////////////
'''

HEADER = Header(frame_id='/base')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 16, PointField.FLOAT32, 1)
          ]
after_gpis_base = pc2.create_cloud(HEADER, fields, new_cloud_4)
cloud_pub_after = rospy.Publisher('gpis_debug_after_base',PointCloud2,queue_size=10)


'''
/////////////////////////////////////////////
start ray_casting and compute the gradient
'''
cloud_out_ = transform_cloud(base_cloud, base_camera)
after_ray, temp_list,distance = call_ray_cast(cloud_out_,center_base,test_cloud,return_index=True)
if after_ray.shape[0] == 0:
    print('out of scope in finite difference')
else:
    print('in scope')

cloud_out_2 = []
for f in range(num_points):
    virtual_link = str(f)
    test_link = chain.forward_pointcloud(current_angles, end_link=virtual_link, base_link='base')
    pose = np.zeros(3)
    pose[0:3] = test_link[0:3, 3].ravel()
    # print(pose)
    cloud_out_2.append(pose)

cloud_out_ = transform_cloud(cloud_out_2, base_camera)
after_ray_2, temp_list_2,distance_2 = call_ray_cast(cloud_out_,center_base,test_cloud,return_index=True)

'''
/////////////////////////////////////////////////////////////////////////////////
current infor gain
'''
var = gpflow.predict_f_full_cov(after_ray_2)
var = var[1]
temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
for i in range(var.shape[0]):
    for j in range(var.shape[1]):
         temp_var[i][j] = var[i][j][0]
         if i == j:
              temp_var[i][j] += 0.1
temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
info_gain = np.linalg.det(temp_var)
info_gain_after = np.log(info_gain) * (0.5)
print('info_gain_before',info_gain_after)
'''
//////////////////////////////////////////////////////////////////////////////////
after rotation infor gain
'''
var = gpflow.predict_f_full_cov(after_ray)
var = var[1]
temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
for i in range(var.shape[0]):
    for j in range(var.shape[1]):
         temp_var[i][j] = var[i][j][0]
         if i == j:
              temp_var[i][j] += 0.1
temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
info_gain = np.linalg.det(temp_var)
info_gain_after = np.log(info_gain) * (0.5)
print('info_gain_after',info_gain_after)


'''
//////////////////////////////////////////////////////////////////////////////////
after transition infor gain
'''
cloud_out_2 = cloud_out_2 + np.array([trans_x,trans_y,trans_z])
cloud_out_ = transform_cloud(cloud_out_2, base_camera)
after_ray_2, temp_list_2,distance_2 = call_ray_cast(cloud_out_,center_base,test_cloud,return_index=True)
var = gpflow.predict_f_full_cov(after_ray_2)
var = var[1]
temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
for i in range(var.shape[0]):
    for j in range(var.shape[1]):
         temp_var[i][j] = var[i][j][0]
         if i == j:
              temp_var[i][j] += 0.1
temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
info_gain = np.linalg.det(temp_var)
info_gain_after = np.log(info_gain) * (0.5)
print('info_gain_trans',info_gain_after)
HEADER = Header(frame_id='/base')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
after_gpis_base1 = pc2.create_cloud(HEADER, fields, cloud_out_2)
cloud_pub_after1 = rospy.Publisher('gpis_debug_trans',PointCloud2,queue_size=10)
'''
/////////////////////////////////////////////////////////////////////////////////
publish pointcloud
'''
rate = rospy.Rate(10)
print('before debug')
while not rospy.is_shutdown():
    cloud_pub.publish(obj_cloud_gpis)
    cloud_pub_after.publish(after_gpis_base)
    cloud_pub_after1.publish(after_gpis_base1)
    rate.sleep()




if __name__ == '__main__':

    rospy.set_param('robot_description_test', baxter_urdf_path)
