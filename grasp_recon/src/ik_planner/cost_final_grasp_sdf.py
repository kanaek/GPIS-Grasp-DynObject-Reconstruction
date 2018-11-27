# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import PyKDL

def final_grasp_sdf_cost(x_des,u,dyn_model,t=-1):
    #wr=dyn_model.wr
    w_c=dyn_model.wc
    
    l_col=0.0
    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    
    for i in range(1,len(c_data)):
        # Check if there are any collisions:
        if(c_data[i]):# If not empty
            
            for j in range(len(c_data[i])):
                # Penetration depth (-ve if not in collision)
                p_d=c_data[i][j][3]

                if(p_d>0.0):# If objects are in collision:
                    l_col+=w_c*(np.abs(p_d+0.5*dyn_model.d_safe)**2)
                elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
                    l_col+=w_c*0.5*(np.abs(-p_d-dyn_model.d_safe)**2)
                else:
                    l_col+=0.0


    return l_col


def final_grasp_sdf_gradient(x_des,u,dyn_model,t=-1):
    #wf=dyn_model.wf
    w_c=dyn_model.wc
    
    gradient_fk=np.zeros(dyn_model.m)
    

    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    

    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    # c_data index differs from actual joint index:
    i_offset=np.array([0,0,0,0,0,0,0,0,-1,
                       1,1,1,
                       1,1,1,
                       1,1,1,
                       2,2])
    for i in range(1,len(c_data)):
        #
        k=i+i_offset[i]
        step_=np.zeros(11)
        if(k<7):
            l_num=k
            f_index=-1
        else:
            f_index=int((k-8)/4)
            l_num=f_index*4-(k-8)
    
        # Check if there are any collisions:
        if(c_data[i]):# If not empty
            
            link_J=dyn_model.allegro.link_jacobian(u,l_num,f_index)[0:3,:]
            j_len=len(np.ravel(link_J[0,:]))
            for j in range(len(c_data[i])):

                # Penetration depth (-ve if not in collision)
                p_d=c_data[i][j][3]

                # compute required change in point position to be collision free:
                point=c_data[i][j][1]
                dir_=c_data[i][j][2]
               
                if(p_d>0.0):# If objects are in collision:
                    l_col=p_d+0.5*dyn_model.d_safe
                    delta_p=point-dir_*l_col
                    delta_link=r_poses[i][0:3]-delta_p                    
                    step_[0:j_len]+=w_c*np.array(-delta_link*link_J).ravel()
                    
                elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
                    #print p_d
                    l_col=0.5*np.abs(-p_d-dyn_model.d_safe)
                    #print l_col
                    # compute required change in link frame:
                    delta_p=point-dir_*l_col

                    delta_link=r_poses[i][0:3]-delta_p                    
                    step_[0:j_len]+=w_c*np.array(-delta_link*link_J).ravel()
                
                else:
                    l_col=0.0
        gradient_fk[0:7]+=step_[:7]

    return gradient_fk

def final_grasp_f_sdf_cost(x_des,u,dyn_model,t=-1):
    #wr=dyn_model.wr
    w_c=dyn_model.wc
    wf=dyn_model.wf    
    l_col=0.0
    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    diff_pose=(x_des-dyn_model.allegro.palm_pose_array(u))
    l_obj=wf*np.sum(np.abs(diff_pose*dyn_model.dimension_wt)**2)

    for i in range(1,len(c_data)):
        # Check if there are any collisions:
        if(c_data[i]):# If not empty
            
            for j in range(len(c_data[i])):
                # Penetration depth (-ve if not in collision)
                p_d=c_data[i][j][3]

                if(p_d>0.0):# If objects are in collision:
                    l_col+=w_c*(np.abs(p_d+0.5*dyn_model.d_safe)**2)
                elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
                    l_col+=w_c*0.5*(np.abs(-p_d-dyn_model.d_safe)**2)
                else:
                    l_col+=0.0


    return l_col+l_obj


def final_grasp_f_sdf_gradient(x_des,u,dyn_model,t=-1):
    wf=dyn_model.wf
    w_c=dyn_model.wc
    
    gradient_fk=np.zeros(dyn_model.m)
    

    joints=list(u)+list(dyn_model.allegro.preshape)
    r_poses=dyn_model.allegro.collision_link_poses(joints)
    

    c_data=dyn_model.c_checker.get_signed_distance(r_poses)
    
    # c_data index differs from actual joint index:
    i_offset=np.array([0,0,0,0,0,0,0,0,-1,
                       1,1,1,
                       1,1,1,
                       1,1,1,
                       2,2])
    for i in range(1,len(c_data)):
        #
        k=i+i_offset[i]
        step_=np.zeros(11)
        if(k<7):
            l_num=k
            f_index=-1
        else:
            f_index=int((k-8)/4)
            l_num=f_index*4-(k-8)
    
        # Check if there are any collisions:
        if(c_data[i]):# If not empty
            
            link_J=dyn_model.allegro.link_jacobian(u,l_num,f_index)[0:3,:]
            j_len=len(np.ravel(link_J[0,:]))
            for j in range(len(c_data[i])):

                # Penetration depth (-ve if not in collision)
                p_d=c_data[i][j][3]

                # compute required change in point position to be collision free:
                point=c_data[i][j][1]
                dir_=c_data[i][j][2]
               
                if(p_d>0.0):# If objects are in collision:
                    l_col=p_d+0.5*dyn_model.d_safe
                    delta_p=point-dir_*l_col
                    delta_link=r_poses[i][0:3]-delta_p                    
                    step_[0:j_len]+=w_c*np.array(-delta_link*link_J).ravel()
                    
                elif(p_d!=0 and -p_d<=dyn_model.d_safe): # If object is within d_safe but not in collision
                    #print p_d
                    l_col=0.5*np.abs(-p_d-dyn_model.d_safe)
                    #print l_col
                    # compute required change in link frame:
                    delta_p=point-dir_*l_col

                    delta_link=r_poses[i][0:3]-delta_p                    
                    step_[0:j_len]+=w_c*np.array(-delta_link*link_J).ravel()
                
                else:
                    l_col=0.0
        gradient_fk[0:7]+=step_[:7]
    diff_pose=(x_des-dyn_model.allegro.palm_pose_array(u))*dyn_model.dimension_wt
    jacobian=dyn_model.allegro.palm_jacobian(u)
    # Gradient: [du dx]
    gradient_fk[0:dyn_model.m]+=-1.0*wf*np.array(np.matrix(diff_pose*dyn_model.position_dimension)*
                                                np.matrix(jacobian)).ravel()

    gradient_fk[4:dyn_model.m]+=-1.0*wf*np.array(np.matrix(diff_pose[3:])*np.matrix(jacobian[3:,4:])).ravel()

    return gradient_fk
