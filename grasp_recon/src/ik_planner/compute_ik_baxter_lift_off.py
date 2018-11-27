# This program computes open loop trajectory for performing in grasp manipulation.
#from cost_ik_lbr4 import *
#from cost_ik_lbr4_self_collision import *

import geometry_msgs.msg
import tf2_ros
# from cost_palm_collision import *
from cost_palm_collision_as_contraint import *
from cost_final_grasp import *
from cost_final_grasp_sdf import *
from cost_ik_baxter_grasp import *
import traj_opt as tjO
import time
import sys
import pickle
import traj_opt_collision as tjO_collsion
import pyOpt_lbr4_vel_constraint as constraint_fn
#import dyn_model
import matplotlib.pyplot as plt

# Find package with allegro model
from rospkg import RosPack
from geometry_msgs.msg import Pose

# import required allegro model files
rp=RosPack()
rp.list()

path=rp.get_path('optimization_baxter')+'/src'

sys.path.insert(0,path)
from kdl_baxter import *
from kdl_baxter_hand_model import *
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
import  pyOpt_cost_function as cost_fn
import  pyOpt_cost_function_collision as cost_fn_collsion
from interpolate_traj import *

sys.path.insert(0,path_opt_pkg+'/scripts')
#from trajectory_viz import *


# path_collision_pkg=rp.get_path('pcl_mesh_tools')+'/python'
# sys.path.insert(0,path_collision_pkg)
# from convex_collision_checker import *
# from convex_collision_checker_client import *
import numpy as np

from ik_plan import ik_solver


import rospy
from geometry_msgs.msg import Pose,PoseArray

class baxter_ik_lift_off:
    def __init__(self,T,delta_t=1.0/60 ):
        self.dyn_model=baxterKinematics(T=T)
        self.dyn_model.allegro=baxter_hand_model(T,delta_t)
        self.T=T
        self.delta_t=delta_t
        self.ik_solver = ik_solver()


    def optimize_final_grasp(self, x_d, j_0, angles, x_des2,hand_j0):
        T = self.T
        self.dyn_model = baxterKinematics(T=T)
        self.dyn_model.allegro = baxter_hand_model(T, T)
        lbr4 = self.dyn_model

        # lbr4.c_checker = convex_collision_checker_client()
        # lbr4.allegro.preshape = hand_j0


        x_des = x_d  # lbr4.end_effector_pose_array(joints_des)
        joints = j_0  # np.random.uniform(lbr4.bounds[0],lbr4.bounds[1])#joints_des.copy()
        lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        lbr4.dimension_wt = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        lbr4.wr = 0.003
        # lbr4.wc = 0.09
        # lbr4.d_safe = 0.01
        lbr4.wf = 40.0
        lbr4.wm = 0.0  # This cost is on midpoint for joint bounds

        max_iter = 7000

        u_input = np.array([joints.copy() for k in range(T + 1)])  # could be improved by trac-ik
        u_input[0] = j_0  # joints.copy()#joints_des.copy()
        u0 = u_input[0]
        x0 = lbr4.end_effector_pose_array(u_input[0])

        # x_des_2 = np.append(np.array(x_des), np.zeros(1))
        x_des_2 = x_des2
        x0_2 = lbr4.end_effector_pose_array(u_input[0], get_quaternion=True)

        # Interpolate x_desired:
        linear_poses = []
        for i in range(T):
            sample_pose = x0 + (x_des - x0) * float(i + 1) / T
            sample_pose[3] = x_des[3]
            sample_pose[4] = x_des[4]
            sample_pose[5] = x_des[5]

            if i == (T - 1):
                sample_pose_2 = x0_2 + (x_des2 - x0_2) * float(i + 1) / T
                l_pose = Pose()
                l_pose.position.x = sample_pose_2[0]
                l_pose.position.y = sample_pose_2[1]
                l_pose.position.z = sample_pose_2[2]
                l_pose.orientation.x = x_des2[3]
                l_pose.orientation.y = x_des2[4]
                l_pose.orientation.z = x_des2[5]
                l_pose.orientation.w = x_des2[6]
                temp_ik = self.ik_solver.ik_solve(l_pose, angles)
                print 'ik_solution', temp_ik[0]
                u_input[i] = temp_ik[0][:]

            linear_poses.append(sample_pose)

        l_pose = Pose()
        l_pose.position.x = x_des2[0]
        l_pose.position.y = x_des2[1]
        l_pose.position.z = x_des2[2]
        l_pose.orientation.x = x_des2[3]
        l_pose.orientation.y = x_des2[4]
        l_pose.orientation.z = x_des2[5]
        l_pose.orientation.w = x_des2[6]
        temp_ik = self.ik_solver.ik_solve(l_pose, angles)
        print 'ik_solution', temp_ik[0]
        u_input[T] = temp_ik[0][:]
        temp_u_final = u_input[T]
        temp_u_first = u_input[0]

        for i in range(T):
            sample_input = temp_u_first + (temp_u_final - temp_u_first) * float(i + 1) / T
            if i <= T/2:
                u_input[i] = sample_input[:]
            else:
                u_input[i] = sample_input[:]
            # print 'sample_input', sample_input


        # Building transformation matrix:


        x_d = linear_poses

        # u0 = u_input[0]
        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}
        # opt_options = {'MAXIT': 7000, 'IPRINT': 0}

        # Initialize opt classes:
        cost_fns = cost_fn.CostFns(final_pose_cost, final_f_pose_cost, final_pose_gradient, final_f_pose_gradient, lbr4, x_d, u0)
        constraint_fns = constraint_fn.ConstraintFns(lbr4, u0, 0.2)
        # Bounds for States and input:
        u_bounds = lbr4.bounds

        # compute mid points for joint positions:
        lbr4.joint_mid_pts = (lbr4.bounds[1] + lbr4.bounds[0]) / 2.0

        # Adding bounds for x:

        low_bounds = []
        up_bounds = []
        for i in range(T):
            low_bounds.extend(u_bounds[0])
            up_bounds.extend(u_bounds[1])

        variable_bounds = [low_bounds, up_bounds]

        opta = tjO.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SNOPT", opt_options)
        # opta = tjO.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SLSQP", opt_options)

        optimizer = opta.optimizer_init()

        u_new, x_new, fstr = opta.optimize(optimizer, u0, x_d, u_input, lbr4)

        x_final = lbr4.end_effector_pose_array(u_new[-1])

        # print('Desired:')
        # print('x_des', x_des)
        # print('x_final', x_final)

        # Interpolate trajectory:
        u_interpol = interpolate_traj(u_new, self.T * 3)

        #if(len(u_new)<self.T):
        #  u_new=interpolate_traj(u_new,self.T+1)

        # print "Final joint angles:"
        # print u_new
        # print len(u_interpol)
        return u_interpol, u_new


    def get_ik_plan(self,x_d,j_0,angles,x_des2):
        arm_j0=j_0[0:7]
        hand_j0=j_0[7:]
        #u_new,u_ik=self.optimize_ik(x_d,arm_j0,angles,x_des2)
        u_new, u_ik = self.optimize_final_grasp(x_d, arm_j0, angles, x_des2,hand_j0)
        print('u_newwwww',u_new)
        return u_ik



