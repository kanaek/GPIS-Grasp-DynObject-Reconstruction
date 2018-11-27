#!/usr/bin/env python
# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import PyKDL
from geometry_msgs.msg import Pose,PoseArray
import math

def to_kdl_frame_rpy(pose):
    return PyKDL.Frame(PyKDL.Rotation.RPY(pose[3], pose[4],
                                                 pose[5]),
                        PyKDL.Vector(pose[0],pose[1],
                                     pose[2]))

def compute_jacobian(dyn_model,l_idx, q_state, c_pt):
    grad = np.zeros((len(q_state),3))
    cpt_mat = np.identity(4)
    cpt_mat[0][3] = c_pt[0]
    cpt_mat[1][3] = c_pt[1]
    cpt_mat[2][3] = c_pt[2]

    #l_mat get the link pose
    l_mat = dyn_model.allegro.link_poses_matrix(q_state,l_idx)

    l_T_p = np.dot(np.linalg.inv(l_mat), cpt_mat)



    eps = 0.3

    p_delta = np.zeros(len(q_state))
    p_pose_array = np.zeros(3)
    n_pose_array = np.zeros(3)
    for i in range(len(q_state)-2):
        p_delta[:] = 0
        p_delta[i] = eps

        # l_mat get the link pose q_state+p_delta
        q_state_plus = q_state + p_delta
        l_mat = dyn_model.allegro.link_poses_matrix(q_state_plus,l_idx)
        p_pose = np.dot(l_mat, l_T_p)
        p_pose_array[0] = p_pose[0, 3]
        p_pose_array[1] = p_pose[1, 3]
        p_pose_array[2] = p_pose[2, 3]

        # l_mat get the link pose q_state-p_delta
        q_state_minus = q_state - p_delta
        l_mat = dyn_model.allegro.link_poses_matrix(q_state_minus,l_idx)
        n_pose = np.dot(l_mat, l_T_p)
        n_pose_array[0] = n_pose[0, 3]
        n_pose_array[1] = n_pose[1, 3]
        n_pose_array[2] = n_pose[2, 3]
        grad[i,:] = (p_pose_array-n_pose_array)/(2*eps)


    return grad

def cost(x_des,u,dyn_model,t=-1):

    w_c=dyn_model.wc
    w_m = dyn_model.wm

    l_col=0.0
    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    pose_arr = PoseArray()
    for i in range(len(r_poses)):
        pose = Pose()
        pose.position.x = r_poses[i][0]
        pose.position.y = r_poses[i][1]
        pose.position.z = r_poses[i][2]
        pose.orientation.x = r_poses[i][3]
        pose.orientation.y = r_poses[i][4]
        pose.orientation.z = r_poses[i][5]
        pose.orientation.w = r_poses[i][6]
        pose_arr.poses.append(pose)

    c_data=dyn_model.collision_checking(pose_arr)
    c_data = c_data.link_data
    '''
    get the difference
    '''
    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]
    l_obj = w_m * np.sum(np.abs(diff_pose * dyn_model.dimension_wt) ** 2)

    for i in range(0, len(c_data)):
        if c_data[i].c_data[0].collision == True:
            l_col += np.abs(c_data[i].c_data[0].distance)


    return [l_obj,l_col]


def cost_gradient(x_des,u,dyn_model,t=-1):
    w_c=dyn_model.wc
    w_m = dyn_model.wm
    
    gradient_fk=np.zeros(dyn_model.m)
    gradient_fk_con = np.zeros(dyn_model.m)
    

    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    pose_arr = PoseArray()
    for i in range(len(r_poses)):
        pose = Pose()
        pose.position.x = r_poses[i][0]
        pose.position.y = r_poses[i][1]
        pose.position.z = r_poses[i][2]
        pose.orientation.x = r_poses[i][3]
        pose.orientation.y = r_poses[i][4]
        pose.orientation.z = r_poses[i][5]
        pose.orientation.w = r_poses[i][6]
        pose_arr.poses.append(pose)

    c_data=dyn_model.collision_checking(pose_arr)
    c_data = c_data.link_data

    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]
    jacobian = dyn_model.allegro.palm_jacobian(u)
    gradient_fk[0:dyn_model.m] += -1.0 * w_m * np.array(np.matrix(diff_pose[0:3]) * np.matrix(jacobian[0:3, 0:])).ravel()

    for i in range(0, len(c_data)):
        if c_data[i].c_data[0].collision == True:
            dir = np.zeros(3)
            dir[:] = c_data[i].c_data[0].dir[:]
            j_mat = compute_jacobian(dyn_model, i, joints, c_data[i].c_data[0].point[0:3])
            j_mat = np.asmatrix(j_mat)
            temp = 1 * math.copysign(1.0, c_data[i].c_data[0].distance)
            temp_2 = np.dot(j_mat, dir)
            temp_2 = np.asarray(temp_2)[0]
            temp_2 = temp_2 * temp
            if i <= 6:
                gradient_fk_con[0:i + 1] += (gradient_fk_con[0:i + 1] + temp_2[0:i + 1])
            else:
                gradient_fk_con[0:7] += (gradient_fk_con[0:7] + temp_2[0:7])



    return [gradient_fk,gradient_fk_con]

def final_cost(x_des,u,dyn_model,t=-1):
    wf=dyn_model.wf
    w_cf = dyn_model.w_cf
    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]

    l_col = 0.0

    l_obj = wf * np.sum(np.abs(diff_pose * dyn_model.dimension_wt) ** 2)


    return [l_obj,l_col]


def final_cost_gradient(x_des,u,dyn_model,t=-1):
    wf=dyn_model.wf
    w_c=dyn_model.wc
    
    gradient_fk=np.zeros(dyn_model.m)
    gradient_fk_con = np.zeros(dyn_model.m)

    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]


    jacobian=dyn_model.allegro.palm_jacobian(u)

    gradient_fk[0:dyn_model.m]+=-1.0*wf*np.array(np.matrix(diff_pose[0:3])*np.matrix(jacobian[0:3,0:])).ravel()




    return [gradient_fk,gradient_fk_con]
