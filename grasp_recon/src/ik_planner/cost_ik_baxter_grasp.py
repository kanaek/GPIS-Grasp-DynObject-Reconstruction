# The cost functions are combined into a single function to allow for faster computation
import numpy as np
from numpy.linalg import inv
import PyKDL

def ik_cost(x_des,u,dyn_model,t=-1):
    wr=dyn_model.wr
    wm=dyn_model.wm
    diff_pose=(x_des-dyn_model.end_effector_pose_array(u))
    l_obj=wr*np.sum(np.abs(diff_pose*dyn_model.dimension_wt))   
    l_mid=wm*np.sum((dyn_model.joint_mid_pts-u)**2)
    return l_obj+l_mid


def ik_cost_gradient(x_des,u,dyn_model,t=-1):
    wr=dyn_model.wr
    wm=dyn_model.wm
    gradient_fk=np.zeros(dyn_model.m)
    diff_pose=(x_des-dyn_model.end_effector_pose_array(u))*dyn_model.dimension_wt
    jacobian=dyn_model.jacobian_full(u)
    # Gradient: [du dx]
    gradient_fk[0:dyn_model.m]=-1.0*wr*np.array(np.matrix(diff_pose*dyn_model.position_dimension)*
                                                np.matrix(jacobian)).ravel()

    gradient_fk[4:dyn_model.m]+=-1.0*wr*np.array(np.matrix(diff_pose[3:])*np.matrix(jacobian[3:,4:])).ravel()

    #l_mid=wm*np.sum(np.abs(dyn_model.joint_mid_pts-u))
    gradient_fk+=-2.0*wm*(dyn_model.joint_mid_pts-u)
    
    return gradient_fk

def ik_final_cost(x_des,u,dyn_model,t=-1): 
    wf=dyn_model.wf
    wm=dyn_model.wm
    #print wf,wm
    diff_pose=(x_des-dyn_model.end_effector_pose_array(u))
    l_obj=wf*np.sum(np.abs(diff_pose*dyn_model.dimension_wt))
    l_mid=wm*np.sum((dyn_model.joint_mid_pts-u)**2)

    return l_obj
    

def ik_final_cost_gradient(x_des,u,dyn_model,t=-1):
    wf=dyn_model.wf
    wm=dyn_model.wm

    gradient_fk=np.zeros(dyn_model.m)
    diff_pose=(x_des-dyn_model.end_effector_pose_array(u))*dyn_model.dimension_wt
    jacobian=dyn_model.jacobian_full(u)
    # Gradient: [du dx]
    gradient_fk[0:dyn_model.m]=-1.0*wf*np.array(np.matrix(diff_pose*dyn_model.position_dimension)*
                                                np.matrix(jacobian)).ravel()

    gradient_fk[4:dyn_model.m]+=-1.0*wf*np.array(np.matrix(diff_pose[3:])*np.matrix(jacobian[3:,4:])).ravel()
    gradient_fk+=-2.0*wm*(dyn_model.joint_mid_pts-u)
    
    return gradient_fk

