#!/usr/bin/env python
import roslib
import rospy
import tf2_ros
#from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
#import moveit_msgs.msg
from geometry_msgs.msg import Point
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

rp=RosPack()
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
import  pyOpt_cost_function as cost_fn
import pyOpt_cost_function_info_gain as cost_fn_info_gain
from kdl_baxter import *
# from kdl_baxter_info_gain import *
#from kdl_lbr4_model import *
#from kdl_lbr4_allegro_model import *
from kdl_baxter_hand_model import *
from collision_check.srv import collsion_check
from collision_check.srv import collision_checker_msg
from visualization_msgs.msg import Marker, MarkerArray
import math


# def compute_jacobian(dyn_model,l_idx, q_state, c_pt):
#     grad = np.zeros((len(q_state),3))
#     cpt_mat = np.identity(4)
#     cpt_mat[0][3] = c_pt[0]
#     cpt_mat[1][3] = c_pt[1]
#     cpt_mat[2][3] = c_pt[2]
#
#     #l_mat get the link pose
#     l_mat = dyn_model.allegro.link_poses_matrix(q_state,l_idx)
#
#     l_T_p = np.dot(np.linalg.inv(l_mat), cpt_mat)
#
#
#
#     eps = 0.1
#
#     p_delta = np.zeros(len(q_state))
#     p_pose_array = np.zeros(3)
#     n_pose_array = np.zeros(3)
#     for i in range(len(q_state)-2):
#         p_delta[:] = 0
#         p_delta[i] = eps
#
#         # l_mat get the link pose q_state+p_delta
#         q_state_plus = q_state + p_delta
#         l_mat = dyn_model.allegro.link_poses_matrix(q_state_plus,l_idx)
#         p_pose = np.dot(l_mat, l_T_p)
#         p_pose_array[0] = p_pose[0, 3]
#         p_pose_array[1] = p_pose[1, 3]
#         p_pose_array[2] = p_pose[2, 3]
#
#         # l_mat get the link pose q_state-p_delta
#         q_state_minus = q_state - p_delta
#         l_mat = dyn_model.allegro.link_poses_matrix(q_state_minus,l_idx)
#         n_pose = np.dot(l_mat, l_T_p)
#         n_pose_array[0] = n_pose[0, 3]
#         n_pose_array[1] = n_pose[1, 3]
#         n_pose_array[2] = n_pose[2, 3]
#         grad[i,:] = (p_pose_array-n_pose_array)/(2*eps)
#
#
#     return grad
#
#
#
rospy.init_node('dynamic_tf_broadcaster')
rs = baxter_interface.RobotEnable(CHECK_VERSION)
print("Enabling robot... ")
rs.enable()
limb_interface = baxter_interface.limb.Limb('left')
# current_angles = np.array([limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()])
# print(current_angles)
# dyn_model = baxterKinematics(T=10)
# dyn_model.allegro = baxter_hand_model(10, 30)
# arm_j0 = current_angles[0:7]
# hand_j0 = current_angles[7:]
# dyn_model.allegro.preshape = hand_j0
# joints=list(arm_j0)+list(dyn_model.allegro.preshape)
# joints.append(-0.020833)
# joints.append(0.020833)
# r_poses=dyn_model.allegro.collision_link_poses(joints)
#
# pose_arr = PoseArray()
# for i in range(len(r_poses)):
#      pose = Pose()
#      pose.position.x = r_poses[i][0]
#      pose.position.y = r_poses[i][1]
#      pose.position.z = r_poses[i][2]
#      pose.orientation.x = r_poses[i][3]
#      pose.orientation.y = r_poses[i][4]
#      pose.orientation.z = r_poses[i][5]
#      pose.orientation.w = r_poses[i][6]
#      pose_arr.poses.append(pose)
# rospy.wait_for_service('/collision_checker')
# collision_checking=rospy.ServiceProxy('/collision_checker',collision_checker_msg)
# res = collision_checking(pose_arr)
# c_data = res.link_data
# # bounding_pub = rospy.Publisher('bounding_box', Marker, queue_size=1)
# # obj_m = Marker()
# # obj_m.header.frame_id = 'base'
# # obj_m.id = 0
# # obj_m.type = 8
# # obj_m.action = Marker.ADD
# # obj_m.scale.x = 0.02
# # obj_m.scale.y = 0.02
# # obj_m.scale.z = 0.02
# # obj_m.color.a = 1
# # obj_m.color.r = 1
# # obj_m.color.g = 0
# # obj_m.color.b = 0
#
# gradient = np.zeros(7)
# for i in range(0, len(c_data)):
#     if c_data[i].c_data[0].collision == True:
#         dir = np.zeros(3)
#         # temp_point = Point()
#         # temp_point.x = c_data[i].c_data[0].point[0]
#         # temp_point.y = c_data[i].c_data[0].point[1]
#         # temp_point.z = c_data[i].c_data[0].point[2]
#         # obj_m.points.append(temp_point)
#
#         dir[:] = c_data[i].c_data[0].dir[:]
#
#         j_mat = compute_jacobian(dyn_model,i, joints, c_data[i].c_data[0].point[0:3])
#         j_mat = np.asmatrix(j_mat)
#         temp = 1*math.copysign(1.0,c_data[i].c_data[0].distance)
#         temp_2 = np.dot(j_mat,dir)
#         temp_2 = np.asarray(temp_2)[0]
#         temp_2 = temp_2 * temp
#         if i<=6:
#             gradient[0:i+1] += (gradient[0:i+1] + temp_2[0:i+1])
#         else:
#             gradient[0:7] += (gradient[0:7] + temp_2[0:7])
#
#         print('temp_2', i,gradient)
#         math.ab
#         # print('res',c_data[i].c_data[0].point)
# # while not rospy.is_shutdown():
# #     bounding_pub.publish(obj_m)



if __name__ == '__main__':
    current_angles = np.array([limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()])
    current_angles_2 = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    temp = np.array([current_angles, current_angles_2])
    current_angles = np.insert(current_angles, 0, 0.66)
    temp = np.insert(temp, 0, current_angles_2)
    temp = temp.reshape(-1, 7)
    print(temp)

    # cost_all = [[45,3,23,68],[34,56,78,90]]
    # np.save('saveModelState', cost_all)
    # temp = np.load('save_best_traj.npy')
    # print(temp)
    temp = np.array([1,2,3])

