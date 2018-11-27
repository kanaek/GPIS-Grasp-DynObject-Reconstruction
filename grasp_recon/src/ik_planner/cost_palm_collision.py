#!/usr/bin/env python
# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import PyKDL

def to_kdl_frame_rpy(pose):
    return PyKDL.Frame(PyKDL.Rotation.RPY(pose[3], pose[4],
                                                 pose[5]),
                        PyKDL.Vector(pose[0],pose[1],
                                     pose[2]))
def cost(x_des,u,dyn_model,t=-1):

    w_c=dyn_model.wc
    w_m = dyn_model.wm

    l_col=0.0
    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
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
    print('difference_pose',diff_pose)

    for i in range(0,len(c_data)):
        # Check if there are any collisions:
        if(c_data[i]):# If not empty

            # for j in range(len(c_data[i])):
            #     # Penetration depth (-ve if not in collision)
            #     p_d=c_data[i][j][3]
            #     if(p_d>0.0):# If objects are in collision:
            #         l_col+=w_c*(p_d+0.5*dyn_model.d_safe)**2
            #     elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
            #         l_col+=w_c*0.5*np.abs(-p_d-dyn_model.d_safe)**2
            #     else:
            #         l_col+=0.0
            for j in range(len(c_data[i])):
                # Penetration depth (-ve if not in collision)
                p_d = c_data[i][j][3]

                if (p_d > 0.0):  # If objects are in collision:
                    l_col += w_c *0.5* p_d**2

                elif (p_d != 0 and -p_d <= dyn_model.d_safe):  # If object is within d_safe but not in collision
                    l_col += w_c * 0.5 * np.abs(-p_d - dyn_model.d_safe)**2
                else:
                    l_col += 0.0


    return l_col+l_obj
    return 0


def cost_gradient(x_des,u,dyn_model,t=-1):
    w_c=dyn_model.wc
    w_m = dyn_model.wm
    
    gradient_fk=np.zeros(dyn_model.m)
    

    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)


    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    temp_u = joints
    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]
    jacobian = dyn_model.allegro.palm_jacobian(u)
    gradient_fk[0:dyn_model.m] += -1.0 * w_m * np.array(np.matrix(diff_pose[0:3]) * np.matrix(jacobian[0:3, 0:])).ravel()


    # c_data index differs from actual joint index:
    # i_offset=np.array([0,0,0,0,0,0,0,0,-1,
    #                    1,1,1,
    #                    1,1,1,
    #                    1,1,1,
    #                    2,2])
    for i in range(0,len(c_data)):
        step_=np.zeros(11)
        if i == 0:
            l_num = 0
            f_index = 0
        elif i == 1:
            l_num = 1
            f_index = 0
        elif i == 2:
            l_num = 0
            f_index = 1
        elif i == 3:
            l_num = 1
            f_index = 1
        # if(i<=6):
        #     l_num=i
        #     f_index=-1
        #     temp_u = u
        # else:
        #     if i == 7:
        #         f_index = -1
        #         l_num = 7
        #         temp_u = u
        #     elif i == 8:
        #         f_index = 0
        #         l_num = 0
        #         temp_u = joints
        #     elif i == 9:
        #         f_index = 0
        #         l_num = 1
        #         temp_u = joints
        #     elif i ==10:
        #         f_index = 1
        #         l_num = 0
        #         temp_u = joints
        #     elif i == 11:
        #         f_index = 1
        #         l_num = 1
        #         temp_u = joints
            #temp_u = joints
        # Check if there are any collisions:
        # if(c_data[i]):# If not empty
        #     link_J=dyn_model.allegro.link_jacobian(temp_u,l_num,f_index)[0:3,:]
        #     j_len=len(np.ravel(link_J[0,:]))
        #     for j in range(len(c_data[i])):
        #
        #         # Penetration depth (-ve if not in collision)
        #         p_d=c_data[i][j][3]
        #
        #         # compute required change in point position to be collision free:
        #         point=c_data[i][j][1]
        #         dir_=c_data[i][j][2]
        #         if(p_d>0.0):# If objects are in collision:
        #             l_col=p_d+0.5*dyn_model.d_safe
        #             delta_p=point-dir_*l_col
        #             delta_link=r_poses[i][0:3]-delta_p
        #             step_[0:j_len]+=w_c*np.array(-delta_p*link_J).ravel()
        #         elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
        #             l_col=0.5*np.abs(-p_d-dyn_model.d_safe)
        #             # compute required change in link frame:
        #             delta_p=point-dir_*l_col
        #
        #             delta_link=r_poses[i][0:3]-delta_p
        #             step_[0:j_len]+=w_c*np.array(-delta_p*link_J).ravel()
        #         else:
        #             l_col=0.0
        if (c_data[i]):  # If not empty

            link_J = dyn_model.allegro.link_jacobian(temp_u, l_num, f_index)[0:3, :]
            j_len = len(np.ravel(link_J[0, :]))
            for j in range(len(c_data[i])):

                # Penetration depth (-ve if not in collision)
                p_d = c_data[i][j][3]

                # compute required change in point position to be collision free:
                point = c_data[i][j][1]
                dir_ = c_data[i][j][2]

                if (p_d > 0.0):  # If objects are in collision:
                    l_col = p_d + 0.5 * dyn_model.d_safe
                    delta_p = point - dir_ * l_col
                    # delta_link = r_poses[i][0:3] - delta_p
                    delta_link = delta_p
                    step_[0:j_len] += w_c * np.array(-delta_link * link_J).ravel()
                elif (p_d != 0 and -p_d <= dyn_model.d_safe):  # If object is within d_safe but not in collision
                    l_col = 0.5 * np.abs(-p_d - dyn_model.d_safe)
                    # compute required change in link frame:
                    delta_p = point - dir_ * l_col
                    # delta_link = r_poses[i][0:3] - delta_p
                    delta_link = delta_p
                    step_[0:j_len] += w_c * np.array(-delta_link * link_J).ravel()

                else:
                    l_col = 0.0
        # gradient_fk[6] += step_[6]
        gradient_fk[0:7]+=step_[:7]

    return gradient_fk

def final_cost(x_des,u,dyn_model,t=-1):
    wf=dyn_model.wf
    w_cf = dyn_model.w_cf
    l_col=0.0
    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]

    # diff_pose=(x_des-dyn_model.allegro.palm_pose_array(u))
    # l_obj=wf*np.sum(np.abs(diff_pose*dyn_model.dimension_wt))
    l_obj = wf * np.sum(np.abs(diff_pose * dyn_model.dimension_wt) ** 2)

    for i in range(0,len(c_data)):
        # Check if there are any collisions:
        if(c_data[i]):# If not empty
            
            # for j in range(len(c_data[i])):
            #     # Penetration depth (-ve if not in collision)
            #     p_d=c_data[i][j][3]
            #
            #     if(p_d>0.0):# If objects are in collision:
            #         l_col+=w_c*(p_d+0.5*dyn_model.d_safe)
            #         print 'in final>0'
            #     elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
            #         l_col+=w_c*0.5*np.abs(-p_d-dyn_model.d_safe)
            #         print 'in final=0'
            #     else:
            #         l_col+=0.0
            for j in range(len(c_data[i])):
                # Penetration depth (-ve if not in collision)
                p_d = c_data[i][j][3]

                if (p_d > 0.0):  # If objects are in collision:
                    l_col += w_cf *0.5* p_d**2

                elif (p_d != 0 and -p_d <= dyn_model.d_safe):  # If object is within d_safe but not in collision
                    l_col += w_cf * 0.5 * np.abs(-p_d - dyn_model.d_safe)**2
                # elif (p_d != 0 and -p_d >= dyn_model.d_safe * 5):
                #     l_col = 0.5 * np.abs(-p_d - dyn_model.d_safe)**2
                else:
                    l_col += 0.0


    # return l_col+l_obj
    return l_col+l_obj


def final_cost_gradient(x_des,u,dyn_model,t=-1):
    wf=dyn_model.wf
    w_c=dyn_model.wc
    
    gradient_fk=np.zeros(dyn_model.m)
    

    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    

    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    kdl_x_des = to_kdl_frame_rpy(x_des)
    kdl_x_current = to_kdl_frame_rpy(dyn_model.end_effector_pose_array(u))
    diff_pose_new = PyKDL.diff(kdl_x_des, kdl_x_current)
    diff_pose = np.zeros(6)
    for i in range(3):
        diff_pose[i] = diff_pose_new[i]
    temp_u = joints
    
    # c_data index differs from actual joint index:
    for i in range(1,len(c_data)):
        step_ = np.zeros(11)
        if i == 0:
            l_num = 0
            f_index = 0
        elif i == 1:
            l_num = 1
            f_index = 0
        elif i == 2:
            l_num = 0
            f_index = 1
        elif i == 3:
            l_num = 1
            f_index = 1
    
        # Check if there are any collisions:
        if(c_data[i]):# If not empty
            
            link_J=dyn_model.allegro.link_jacobian(temp_u,l_num,f_index)[0:3,:]
            j_len=len(np.ravel(link_J[0,:]))
            print('j_len',j_len)
            for j in range(len(c_data[i])):

                # Penetration depth (-ve if not in collision)
                p_d=c_data[i][j][3]

                # compute required change in point position to be collision free:
                point=c_data[i][j][1]
                dir_=c_data[i][j][2]
               
                if(p_d>0.0):# If objects are in collision:
                    l_col=p_d+0.5*dyn_model.d_safe
                    delta_p=point-dir_*l_col
                    # delta_link=r_poses[i][0:3]-delta_p
                    delta_link = delta_p
                    step_[0:j_len]+=wf*np.array(-delta_link*link_J).ravel()
                elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
                    l_col=0.5*np.abs(-p_d-dyn_model.d_safe)
                    # compute required change in link frame:
                    delta_p=point-dir_*l_col

                    # delta_link=r_poses[i][0:3]-delta_p
                    delta_link = delta_p
                    step_[0:j_len]+=wf*np.array(-delta_link*link_J).ravel()
                
                # elif(p_d!=0 and -p_d>=dyn_model.d_safe*5):
                #     l_col = 0.5 * np.abs(-p_d - dyn_model.d_safe)
                #     # compute required change in link frame:
                #     delta_p = point - dir_ * l_col
                #
                #     # delta_link = r_poses[i][0:3] - delta_p
                #     delta_link = delta_p
                #     step_[0:j_len] += wf * np.array(-delta_link * link_J).ravel()
                else:
                    l_col=0.0
        # gradient_fk[0:7]+=step_[:7]
        gradient_fk[0:7] += step_[0:7]

    jacobian=dyn_model.allegro.palm_jacobian(u)
    # Gradient: [du dx]
    # gradient_fk[0:dyn_model.m]+=-1.0*wf*np.array(np.matrix(diff_pose*dyn_model.position_dimension)*
    #                                             np.matrix(jacobian)).ravel()
    #
    gradient_fk[0:dyn_model.m]+=-1.0*wf*np.array(np.matrix(diff_pose[0:3])*np.matrix(jacobian[0:3,0:])).ravel()

    return gradient_fk
