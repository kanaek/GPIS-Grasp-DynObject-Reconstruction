#!/usr/bin/env python
# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import tf
import rospy
import PyKDL
import baxter_interface
from baxter_interface import CHECK_VERSION
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros

from rospkg import RosPack
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
rp=RosPack()
rp.list()

'''
l_gripper_r_finger_joint:lower="-0.020833" upper="0.0"
l_gripper_l_finger_joint:lower="0.0" upper="0.020833"
0.041666
camera_rgb_optical_frame
'''
import sys
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
from kdl_baxter import *
from kdl_baxter_hand_model import *
from tabletop_obj_segmentation.srv import SegmentGraspObject
from tabletop_obj_segmentation.srv import GPIS_INFO
import geometry_msgs.msg
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
# import GPflow
from optimization_baxter.srv import raycast
from pykdl_utils.kdl_kinematics_pointcloud import KDLKinematics_pointcloud
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
import GPy as gp
from geometry_msgs.msg import Point

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x,t.transform.translation.y,
                                     t.transform.translation.z))

def transform_cloud(cloud, transform,color=3):
    t_kdl = transform_to_kdl(transform)
    points_out = []
    for p_in in cloud:
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        if color==3:
            points_out.append((p_out[0], p_out[1], p_out[2]))
        else:
            points_out.append((p_out[0], p_out[1], p_out[2],p_in[3]))
    return points_out


limb = 'left'

init_angle = [-0.8628641931855452, -0.26000974354657763, 0.6239466854723921, 0.8421554525490922, -0.6891408689575221, 1.1010147105047556, 0.24236896448589537]
rospy.init_node("grasp_client222")
rs = baxter_interface.RobotEnable(CHECK_VERSION)
print("Enabling robot... ")
rs.enable()
limb_interface = baxter_interface.limb.Limb(limb)
current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
from urdf_parser_py.urdf import Robot
baxter_urdf_path=rp.get_path('baxter_description')
baxter_urdf_path = baxter_urdf_path+'/urdf/baxter2_correct_2.urdf'
urdf_file = file(baxter_urdf_path, 'r')
robot = Robot.from_xml_string(urdf_file.read())
chain = KDLKinematics_pointcloud(robot, 'base', 'left_gripper','left_gripper')

print('current',current_angles)

import sensor_msgs.point_cloud2 as pc2

test_cloud = np.load('new_xx.npy')
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = tf_buffer.lookup_transform('left_gripper',
                                           'camera_rgb_optical_frame',  # source frame
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.0))  # wait for 1 second

transform_rgb_camera = tf_buffer.lookup_transform('camera_link',
                                       'camera_rgb_optical_frame',  # source frame
                                        rospy.Time(0),  # get the tf at first available time
                                        rospy.Duration(1.0))  # wait for 1 second

'''
/////////////////////////////////////////////////////////////////////////////////////
only for testing ray-casting purpose
'''
cloud_out_camera = transform_cloud(test_cloud, transform_rgb_camera)
HEADER = Header(frame_id='/camera_link')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_camera)
point_center = Point()
point_center.x = 0.0
point_center.y = 0.0
point_center.z = 0.0
# rospy.wait_for_service('raycast')
# raycast_srv = rospy.ServiceProxy('raycast', raycast)
# raycast_resp = raycast_srv(obj_cloud_gpis,point_center)
# temp_list = raycast_resp.array
rate = rospy.Rate(10)
# print('temp_list',temp_list)

cloud_pub = rospy.Publisher('gpis_debug',PointCloud2,queue_size=10)
after_ray1 = []
for i in range(len(cloud_out_camera)):
    temp = list(cloud_out_camera[i][0:3])
    temp.append(test_cloud[i][3])
    after_ray1.append(temp)

after_ray1 = np.array(after_ray1)
HEADER = Header(frame_id='/camera_link')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgb', 16, PointField.FLOAT32, 1)
          ]
after_gpis = pc2.create_cloud(HEADER, fields, after_ray1)
while not rospy.is_shutdown():
    cloud_pub.publish(after_gpis)
    rate.sleep()
'''
////////////////////////////////////////////////////////////////////////////////////////
'''

cloud_out = transform_cloud(test_cloud, transform)
cloud_center = []
for i in range(len(cloud_out)):
    if i == 0:
        child_name_cloud = str(i)
        pose = cloud_out[i]
        cloud_center = np.array(list(pose))
        # print(i)
        chain.add_new_points_to_tree(pose, child_name_cloud)
    else:
        child_name_cloud = str(i)
        pose = cloud_out[i]
        cloud_center = cloud_center + np.array(list(pose))
        chain.add_new_points_to_tree(pose, child_name_cloud)

cloud_center = np.dot(cloud_center,1.0/len(cloud_out))
cloud_center = tuple(cloud_center)
chain.add_new_points_to_tree(cloud_center, 'cloud_center')
print('cloud_center',cloud_center)

transform = tf_buffer.lookup_transform('camera_link',
                                           'base',  # source frame
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.0))  # wait for 1 second

cloud_out_ = []
for i in range(len(cloud_out)):
    virtual_link = str(i)
    test_link = chain.forward_pointcloud(init_angle, end_link=virtual_link, base_link='base')
    pose = np.zeros(3)
    pose[0:3] = test_link[0:3,3].ravel()
    cloud_out_.append(pose)


cloud_out_ = transform_cloud(cloud_out_, transform)

cloud_out_center = []
test_link = chain.forward_pointcloud(init_angle, end_link='cloud_center', base_link='base')
pose = np.zeros(3)
pose[0:3] = test_link[0:3, 3].ravel()
cloud_out_center.append(pose)
cloud_out_center = transform_cloud(cloud_out_center, transform)
point_center = Point()
point_center.x = cloud_out_center[0][0]
point_center.y = cloud_out_center[0][1]
point_center.z = cloud_out_center[0][2]
print(point_center)


HEADER = Header(frame_id='/camera_link')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
obj_cloud_gpis = pc2.create_cloud(HEADER,fields, cloud_out_)

rospy.wait_for_service('raycast')
raycast_srv = rospy.ServiceProxy('raycast', raycast)
raycast_resp = raycast_srv(obj_cloud_gpis,point_center)
# ray_cast_cloud = pc2.read_points(raycast_resp.ray_cast_cloud, skip_nans=True)
# ray_cast_cloud = list(ray_cast_cloud)
temp_list = raycast_resp.array
distance = raycast_resp.distance
after_ray = []
for i in range(len(temp_list)):
    temp = list(test_cloud[temp_list[i]][0:3])
    after_ray.append(temp)
after_ray = np.array(after_ray)
print('distance',distance)


HEADER = Header(frame_id='/camera_rgb_optical_frame')
fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)
          ]
after_gpis = pc2.create_cloud(HEADER,fields, after_ray)




# k = GPflow.kernels.RBF(input_dim=3, lengthscales=0.2)
#msstate = np.load('saveModelState.npy')
X = np.load('X.npy')
Y = np.load('Y.npy')
width = np.load('width.npy')
height = np.load('height.npy')
depth = np.load('depth.npy')
# meanf = GPflow.mean_functions.Ellipsoidal(width,height,depth)
# m2 = GPflow.gpr.GPR(X, Y, kern=k,mean_function=meanf)
# m2.likelihood.variance = 0.1
#m2.set_state(msstate)
k_2 = gp.kern.RBF(input_dim=3, variance=3.0, lengthscale=0.10)
gpy = gp.models.GPRegression(X, Y, kernel=k_2)
gpy.likelihood.variance = 0.1


# var = m2.predict_f_full_cov(after_ray)
# var = var[1]
# temp_var = np.zeros((var.shape[0],var.shape[1]),dtype=np.float)
# for i in range(var.shape[0]):
#     for j in range(var.shape[1]):
#         temp_var[i][j] = var[i][j][0]
#         if i == j:
#             temp_var[i][j] += 0.1
# info_gain_fake = np.linalg.slogdet(temp_var)
# temp_var = np.dot(2.0*np.pi*np.exp(1),temp_var)
# info_gain = np.linalg.det(temp_var)
# info_gain = np.log(info_gain)*0.5
rate = rospy.Rate(10)
cloud_pub = rospy.Publisher('gpis_debug',PointCloud2,queue_size=10)
while not rospy.is_shutdown():
    cloud_out_ = []
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    for i in range(len(cloud_out)):
        virtual_link = str(i)
        test_link = chain.forward_pointcloud(current_angles, end_link=virtual_link, base_link='base')
        pose = np.zeros(3)
        pose[0:3] = test_link[0:3, 3].ravel()
        cloud_out_.append(pose)

    cloud_out_1 = transform_cloud(cloud_out_, transform)

    HEADER = Header(frame_id='/camera_link')
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)
              ]
    obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_1)
    point_center = Point()
    point_center.x = 0.0
    point_center.y = 0.0
    point_center.z = 0.0
    rospy.wait_for_service('raycast')
    raycast_srv = rospy.ServiceProxy('raycast', raycast)
    raycast_resp = raycast_srv(obj_cloud_gpis,point_center)
    temp_list = raycast_resp.array
    temp_list = list(set(temp_list))
    after_ray = []
    for i in range(len(temp_list)):
        after_ray.append(list(test_cloud[temp_list[i]][0:3]))
    after_ray = np.array(after_ray)
    var = gpy.predict(after_ray,full_cov=True)
    var = var[1]
    temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
    info_gain = np.linalg.det(temp_var)
    info_gain = np.log(info_gain) * 0.5
    print('conditional entropy',info_gain)
    # print('temp_list',temp_list)

    after_ray1 = []
    for i in range(len(temp_list)):
        temp = list(cloud_out_[temp_list[i]][0:3])
        # temp.append(test_cloud[temp_list[i]][3])
        after_ray1.append(temp)


    after_ray1 = np.array(after_ray1)
    HEADER = Header(frame_id='/base')
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)
              # PointField('rgb', 16, PointField.FLOAT32, 1)
              ]
    after_gpis = pc2.create_cloud(HEADER, fields, after_ray1)
    cloud_pub.publish(after_gpis)
    rate.sleep()





def info_gain_cost(x_des,u,dyn_model):
    wm = dyn_model.wm
    cloud_out_1 = dyn_model.FK_pointcloud(u)
    after_ray = dyn_model.call_ray_cast(cloud_out_1,return_index=False)
    var = dyn_model.gpflow.predict_f_full_cov(after_ray)
    var = var[1]
    temp_var = np.zeros((var.shape[0],var.shape[1]),dtype=np.float)
    for i in range(var.shape[0]):
        for j in range(var.shape[1]):
            temp_var[i][j] = var[i][j][0]
            if i == j:
                temp_var[i][j] += 0.1
    # info_gain = np.linalg.slogdet(temp_var)
    temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
    info_gain = np.linalg.det(temp_var)
    info_gain = np.log(info_gain) * 0.5
    return info_gain*(-wm)


def info_gain_cost_gradient(x_des,u,dyn_model):
    wm = dyn_model.wm
    cloud_out_1 = dyn_model.FK_pointcloud(u)
    after_ray, temp_list = dyn_model.call_ray_cast(cloud_out_1,return_index=True)
    var = dyn_model.gpflow.predict_f_full_cov(after_ray)
    var = var[1]
    temp_var = np.zeros((var.shape[0],var.shape[1]),dtype=np.float)
    for i in range(var.shape[0]):
        for j in range(var.shape[1]):
            temp_var[i][j] = var[i][j][0]
            if i == j:
                temp_var[i][j] += 0.1
    temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
    inv_matrix = np.linalg.inv(temp_var)
    transpose_matrix = np.transpose(inv_matrix)
    grad = dyn_model.gpy.predictive_gradients(after_ray)
    grad = grad[1]
    temp_grad = np.dot(transpose_matrix, grad)
    final_grad = []
    for i in range(len(temp_list)):
        virtual_link = str(temp_list[i])
        test_jacobian = dyn_model.chain.jacobian_pointcloud(u, pos=None, base_link='base', end_link=virtual_link)
        test_jacobian_point = test_jacobian[0:3, :]
        final = np.dot([temp_grad[i]], test_jacobian_point)
        final_grad.append(final)
    final_grad = np.dot(np.mean(final_grad, axis=0), 0.5)
    final_grad = np.dot(final_grad, wm)
    # for i in len(temp_list):
    #     virtual_link = str(temp_list[i])
    #     test_jacobian = dyn_model.chain.jacobian_pointcloud(u, pos=None, base_link='base', end_link=virtual_link)
    #     test_jacobian_point = test_jacobian[0:3, :]
    # test_jacobian = dyn_model.jacobian_pointcloud_center(u)
    # test_jacobian_point = test_jacobian[0:3, :]
    # final_grad = np.dot(temp_grad,test_jacobian_point)
    # final_grad = np.dot(np.mean(final_grad, axis=0), 0.02)
    return final_grad

# def final_pose_cost(x_des,u,dyn_model,t=-1):
#     wr = dyn_model.wr
#     wm = dyn_model.wm
#     diff_pose = (x_des - dyn_model.end_effector_pose_array(u))
#     l_obj = wr * np.sum(np.abs(diff_pose*dyn_model.dimension_wt)**2)
#     l_mid = wm * np.sum((dyn_model.joint_mid_pts - u) ** 2)
#     #return l_obj
#     return l_obj + l_mid
#
#
# def final_pose_gradient(x_des,u,dyn_model,t=-1):
#     wr = dyn_model.wr
#     wm = dyn_model.wm
#     gradient_fk = np.zeros(dyn_model.m)
#     diff_pose = np.abs(x_des - dyn_model.end_effector_pose_array(u))*dyn_model.dimension_wt
#     jacobian = dyn_model.jacobian_full(u)
#     # Gradient: [du dx]
#     gradient_fk[0:dyn_model.m] = -1.0 * wr * np.array(np.matrix(diff_pose * dyn_model.position_dimension) *
#                                                       np.matrix(jacobian)).ravel()
#
#     gradient_fk[4:dyn_model.m] += -1.0 * wr * np.array(np.matrix(diff_pose[3:]) * np.matrix(jacobian[3:, 4:])).ravel()
#
#     #l_mid=wm*np.sum(np.abs(dyn_model.joint_mid_pts-u))
#     gradient_fk += -1.0 * wm * (dyn_model.joint_mid_pts - u)
#
#     return gradient_fk


def final_f_pose_cost(x_des,u,dyn_model,t=-1):
    wf = dyn_model.wf
    wm = dyn_model.wm
    # print wf,wm
    diff_pose = (x_des - dyn_model.end_effector_pose_array(u))
    l_obj = wf * np.sum(np.abs(diff_pose*dyn_model.dimension_wt)**2)
    l_mid = wm * np.sum((dyn_model.joint_mid_pts - u) ** 2)

    return l_obj

def final_f_pose_gradient(x_des,u,dyn_model,t=-1):
    wf = dyn_model.wf
    wm = dyn_model.wm

    gradient_fk = np.zeros(dyn_model.m)
    diff_pose = np.abs(x_des - dyn_model.end_effector_pose_array(u))*dyn_model.dimension_wt
    jacobian = dyn_model.jacobian_full(u)
    # Gradient: [du dx]
    gradient_fk[0:dyn_model.m] = -1.0 * wf * np.array(np.matrix(diff_pose * dyn_model.position_dimension) *
                                                      np.matrix(jacobian)).ravel()

    gradient_fk[4:dyn_model.m] += -1.0 * wf * np.array(np.matrix(diff_pose[3:]) * np.matrix(jacobian[3:, 4:])).ravel()
    #gradient_fk += -2.0 * wm * (dyn_model.joint_mid_pts - u)

    return gradient_fk


