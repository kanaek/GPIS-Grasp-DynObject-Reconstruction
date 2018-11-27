#!/usr/bin/env python
# This program computes open loop trajectory for performing in grasp manipulation.
#from cost_ik_lbr4 import *
#from cost_ik_lbr4_self_collision import *

import geometry_msgs.msg
import tf2_ros
import tf
from cost_palm_collision import *
from cost_final_grasp import *
# from cost_info_gain import *
#from cost_final_grasp_sdf import *
#from cost_ik_baxter_grasp import *
import traj_opt as tjO
import time
import sys
import pickle
import traj_opt as tjO
import pyOpt_lbr4_vel_constraint as constraint_fn
from tabletop_obj_segmentation.srv import GPIS_INFO
from tabletop_obj_segmentation.srv import SegmentGraspObject
import sensor_msgs.point_cloud2 as pc2
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
# from kdl_baxter_info_gain import *
from kdl_baxter_info_gain_one import *
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
import  pyOpt_cost_function as cost_fn
import  pyOpt_cost_function_info_gain as cost_fn_info_gain
from interpolate_traj import *

sys.path.insert(0,path_opt_pkg+'/scripts')
#from trajectory_viz import *


# path_collision_pkg=rp.get_path('pcl_mesh_tools')+'/python'
# sys.path.insert(0,path_collision_pkg)
# #from convex_collision_checker import *
# from convex_collision_checker_client import *


from ik_plan import ik_solver


import rospy
from geometry_msgs.msg import Pose,PoseArray,Point
import numpy as np
import pyOpt
import PyKDL


class baxter_ik_info:
    def __init__(self,T,delta_t=1.0/60 ):
        self.dyn_model=baxterKinematics(T=T)
        self.dyn_model.allegro=baxter_hand_model(T,delta_t)
        self.T=T
        self.delta_t=delta_t
        self.ik_solver = ik_solver()
        self.test_cloud = None


    def optimize_final_grasp_sdf(self, x_d, u_ik, hand_j0):


        T = self.T
        self.dyn_model = baxterKinematics(T=T)
        self.dyn_model.allegro = baxter_hand_model(T, 60)
        lbr4 = self.dyn_model

        lbr4.c_checker = convex_collision_checker_client()

        x_des = x_d
        # joints=np.random.uniform(lbr4.bounds[0],lbr4.bounds[1])#joints_des.copy()
        # lbr4.dimension=np.array([10.0,10.0,10.0,0.0,0.0,0.0])
        lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        lbr4.dimension_wt = np.array([1.0, 1.0, 1.0, 0.5, 0.5, 0.5])

        lbr4.wr = 0.01
        lbr4.wc = 100.0
        #lbr4.wsafe = 1.0
        lbr4.wf = 70.0
        #lbr4.wm = 0.01  # This cost is on midpoint for joint bounds
        lbr4.d_safe = 0.01  # safe limit for collision checking

        max_iter = 30

        u_input = u_ik  # np.array([joints.copy() for k in range(T+1)])

        u0 = u_input[0]
        x0 = lbr4.end_effector_pose_array(u_input[0])
        # Interpolate x_desired:
        linear_poses = []
        for i in range(T):
            sample_pose = x0 + (x_des - x0) * float(i + 1) / T
            # Building transformation matrix:
            linear_poses.append(sample_pose)

        x_d = linear_poses

        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}

        # Initialize opt classes:
        #cost_fns = cost_fn.CostFns(final_grasp_sdf_cost, final_grasp_f_sdf_cost, final_grasp_sdf_gradient, final_grasp_f_sdf_gradient, lbr4, x_d, u0)
        cost_fns = cost_fn.CostFns(cost, final_cost, cost_gradient, final_cost_gradient, lbr4, x_d, u0)
        constraint_fns = constraint_fn.ConstraintFns(lbr4, u0, 0.2)
        # Bounds for States and input:
        u_bounds = lbr4.bounds

        # compute mid points for joint positions:
        lbr4.joint_mid_pts = (lbr4.bounds[1] + lbr4.bounds[0]) / 2.0

        lbr4.allegro.preshape = hand_j0
        # Adding bounds for x:

        low_bounds = []
        up_bounds = []
        for i in range(T):
            low_bounds.extend(u_bounds[0])
            up_bounds.extend(u_bounds[1])

        variable_bounds = [low_bounds, up_bounds]

        opta = tjO.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SNOPT", opt_options)

        optimizer = opta.optimizer_init()
        # allegro_model=dyn_model.fkModel(robot_model)
        u_new, x_new = opta.optimize(optimizer, u0, x_d, u_input, lbr4)

        x_final = lbr4.end_effector_pose_array(u_new[-1])

        print 'Desired: '
        print x_des
        print x_final

        lbr4.c_checker.collision_checker.close()

        # Interpolate trajectory:
        u_interpol = interpolate_traj(u_new, T * 3)
        print "Final joint angles:"
        print u_new[-1]
        print 'u_interpolate', u_interpol
        return u_interpol, u_new

    def optimize_info_gain_one(self, x_d, j_0, angles, x_des2):
        self.dyn_model_2 = baxterKinematics_info_gain(T=1)
        self.dyn_model_2.init_transform_pointcloud_kdl()
        self.dyn_model_2.init_gpflow_gpy()
        print('finish init gpflow and gpy')
        lbr4 = self.dyn_model_2
        lbr4.wm = 0.2  # This cost is on midpoint for joint bounds
        lbr4.wg = 0.2
        lbr4.cone = 0.8
        # lbr4.finite_difference = [0.0348,0.0348,0.0348,0.0348,0.0348,0.0348,0.0348]
        lbr4.finite_difference = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        lbr4.trans = np.array([0.2,0.2,0.2])
        lbr4.rot = np.array([1, 1, 1])
        lbr4.finite_array[:] = 0.01 #define the finite difference step size
        max_iter = 20

        def rad_to_degree(degree):
            return degree* 180 / np.pi

        def to_kdl_frame_quaternion(pose):
            return PyKDL.Frame(PyKDL.Rotation.Quaternion(pose[3], pose[4],
                                                         pose[5], pose[6]),
                               PyKDL.Vector(pose[0], pose[1],
                                            pose[2]))

        def to_kdl_frame_rpy(pose):
            return PyKDL.Frame(PyKDL.Rotation.RPY(pose[3], pose[4],
                                                  pose[5]),
                               PyKDL.Vector(pose[0], pose[1],
                                            pose[2]))

        def info_gain_cost(u, dyn_model):

            info_gain = 0
            u = u['u']
            cloud_center = dyn_model.FK_pointcloud_center(u)
            wm = dyn_model.wm

            cloud_out_1 = dyn_model.FK_pointcloud(u)

            after_ray,distance = dyn_model.call_ray_cast(cloud_out_1, cloud_center,return_index=False)

            if after_ray.shape[0]==0:
                print('out of scope in function')
                return 0,distance
            before = rospy.get_time()
            # var = dyn_model.gpflow.predict_f_full_cov(after_ray)
            var = dyn_model.gpy.predict(after_ray, full_cov=True)
            var = var[1]

            temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
            info_gain = np.linalg.det(temp_var)
            info_gain = np.log(info_gain) * 0.5
            after = rospy.get_time()
            print("Current time ", (after - before))
            print('info_gain',info_gain * (wm))

            # print('distance', distance)
            return info_gain * (wm),distance

        def info_gain_cost_gradient(u, dyn_model):
            u = u['u']
            cloud_center = dyn_model.FK_pointcloud_center(u)

            center = dyn_model.chain.forward_pointcloud(u, end_link='cloud_center', base_link='base')
            pose = np.zeros(7)
            pose[0:3] = center[0:3, 3].ravel()
            center_base = pose[0:3]


            temp_u = np.zeros(6)
            temp_u[:] = 0.0
            wg = dyn_model.wg
            cloud_out_2 = []
            for f in range(dyn_model.num_points):
                virtual_link = str(f)
                test_link = dyn_model.chain.forward_pointcloud(u, end_link=virtual_link, base_link='base')
                pose = np.zeros(3)
                pose[0:3] = test_link[0:3, 3].ravel()
                cloud_out_2.append(pose)
            cloud_out_ = dyn_model.transform_cloud(cloud_out_2, dyn_model.base_camera)
            after_ray, temp_list_2, distance = dyn_model.call_ray_cast(cloud_out_, cloud_center, return_index=True)
            if after_ray.shape[0] == 0:
                # print('out of scope before')
                info_gain_before = 0.0
                # return np.array([[0., 0., 0., 0., 0., 0., 0.]])
            else:
                var = dyn_model.gpy.predict(after_ray, full_cov=True)
                var = var[1]
                temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
                info_gain = np.linalg.det(temp_var)
                info_gain_before = np.log(info_gain) * (0.5)

            jacobian = dyn_model.chain.jacobian_pointcloud(u, pos=None, base_link='base', end_link='cloud_center')

            fake_distance = np.array([distance, distance, distance])
            gradient_sdf = fake_distance * jacobian[0:3, :]
            gradient_sdf = np.array(gradient_sdf)[0, :]


            for i in range(3):
                difference = np.zeros(3)
                difference[i] = dyn_model.trans[i]
                cloud_out_i = np.copy(cloud_out_2)
                cloud_out_i = cloud_out_i + difference
                cloud_out_ = dyn_model.transform_cloud(cloud_out_i, dyn_model.base_camera)

                after_ray, temp_list_2, distance_2 = dyn_model.call_ray_cast(cloud_out_, cloud_center,return_index=True)
                if after_ray.shape[0] == 0:
                    print('transition difference out of scope',i)
                    info_gain_after = 0.0
                    grad = (info_gain_after - info_gain_before)/dyn_model.trans[i]
                    temp_u[i] = grad
                    continue
                var = dyn_model.gpy.predict(after_ray, full_cov=True)
                var = var[1]
                temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
                info_gain = np.linalg.det(temp_var)
                info_gain_after = np.log(info_gain) * (0.5)
                grad = (info_gain_after - info_gain_before) / dyn_model.trans[i]
                temp_u[i] = grad

            for i in range(3):
                difference = np.zeros(3)
                difference[i] = dyn_model.rot[i]

                cloud_out_i = np.copy(cloud_out_2)
                cloud_out_i = cloud_out_i - np.array(center_base)

                rot = center[0:3, 0:3]
                new_cloud = []
                for k in range(len(cloud_out_i)):
                    trans = np.transpose(cloud_out_i[k])
                    temp_rot_cloud = np.dot(rot, trans)
                    temp_rot_cloud = np.asarray(temp_rot_cloud)
                    new_cloud.append(temp_rot_cloud[0])

                new_cloud = list(new_cloud)


                R = tf.transformations.euler_matrix(difference[0], difference[1], difference[2],'sxyz')
                rot = R[0:3, 0:3]
                rotate_cloud = []
                for j in range(len(new_cloud)):
                    temp_rot_cloud = np.dot(rot, new_cloud[j])
                    temp_rot_cloud = np.asarray(temp_rot_cloud)
                    rotate_cloud.append(temp_rot_cloud)

                rotate_cloud = np.array(rotate_cloud)
                # rotate_cloud = rotate_cloud + np.array(center_base)

                rot = center[0:3, 0:3]
                rot_inv = np.linalg.inv(rot)
                base_cloud = []
                for m in range(len(rotate_cloud)):
                    temp_rot_cloud = np.dot(rot_inv, rotate_cloud[m])
                    temp_rot_cloud = np.asarray(temp_rot_cloud)
                    base_cloud.append(temp_rot_cloud[0])
                base_cloud = np.array(base_cloud)
                base_cloud = base_cloud + np.array(center_base)


                cloud_out_ = dyn_model.transform_cloud(base_cloud, dyn_model.base_camera)
                after_ray, temp_list_2, distance_2 = dyn_model.call_ray_cast(cloud_out_, cloud_center,
                                                                             return_index=True)
                if after_ray.shape[0] == 0:
                    print('finite difference out of scope in rotation',i)
                    info_gain_after = 0.0
                    grad = (info_gain_after - info_gain_before) / dyn_model.rot[i]
                    temp_u[i+3] = grad
                    continue
                var = dyn_model.gpy.predict(after_ray, full_cov=True)
                var = var[1]
                temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
                info_gain = np.linalg.det(temp_var)
                info_gain_after = np.log(info_gain) * (0.5)
                grad = (info_gain_after - info_gain_before) / dyn_model.rot[i]

                temp_u[i+3] = grad




            gradient = np.dot(temp_u,jacobian)
            gradient = np.asarray(gradient)
            gradient = gradient[0]
            gradient[0:7] = gradient[0:7]*wg
            # ubound0 = np.array([1.70167994, 2.147, 3.05417994, 0.05, 3.059,1.57079633, 3.059])
            # ubound1 = np.array([1.70167994, 1.047, 3.05417994, 2.618, 3.059, 2.094, 3.059])
            print('gradient', gradient)
            # print('temp_u', temp_u)

            return [gradient],[gradient_sdf]


        def objfunc(q):
            f,distance = info_gain_cost(q,lbr4)
            f_final = f
            # g = list(self.constraint_fns.constraint(q))  # +list(self.robot_fns.rolling_constraint(q))
            g=[distance]
            fail = 0

            return f_final,g,fail

        def grad_obj_func(q,f,g):
            g_obj,g_sdf = np.array(info_gain_cost_gradient(q,lbr4))
            # print('g_obj',g_obj)
            # print('g_sdf', g_sdf)
            # g_con = [np.array(g_sdf)]`
            # print(g_obj[0])
            g_con=[]
            fail = 0
            # print g_obj
            return g_obj,g_sdf,fail

        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0, 'Problem Type':'Maximize'}
        opt_prob=pyOpt.Optimization('TP37 Constrained Problem',objfunc,use_groups=True)
        opt_prob.addObj('f')
        u_bounds = lbr4.bounds
        print(u_bounds[0])
        # opt_prob.addVarGroup('u',len(q0),type='c',value=q0,upper=self.bounds[1],lower=self.bounds[0])
        j_0[:,6] = j_0[0,6]
        j_0[9][6] = j_0[9][6]+ 2.2
        print('j_0',j_0[9])
        opt_prob.addVarGroup('u',len(j_0[0]), 'c', lower=u_bounds[0], upper=u_bounds[1], value=j_0[9])
        # opt_prob.addCon('g', 'i',upper=-0.01)

        method=getattr(pyOpt,"SNOPT")
        optimizer=method(options=opt_options)

        [fstr,xstr,inform]=optimizer(opt_prob,sens_type=grad_obj_func) #sens_type could be finite differences. sens_type='FD'
        # [fstr, xstr, inform] = optimizer(opt_prob,sens_type='FD')  # sens_type could be finite differences. sens_type='FD'
        xstr = np.array(xstr)
        print(xstr)

        return [xstr],[xstr]




    def optimize_mov_obj_info_gain(self, x_d, j_0, angles, x_des2):

        T = self.T
        self.dyn_model_2 = baxterKinematics_info_gain(T=T)
        self.dyn_model_2.init_transform_pointcloud_kdl()
        self.dyn_model_2.init_gpflow_gpy()
        print('finish init')
        print('finish init gpflow and gpy')
        lbr4 = self.dyn_model_2
        # lbr4.init_transform_pointcloud_kdl()

        x_des = x_d
        joints = j_0
        lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        lbr4.dimension_wt = np.array([1.0, 1.0, 0.7, 0.3, 0.3, 0.3])

        lbr4.wr = 0.003
        lbr4.wc = 0.0
        lbr4.wf = 0.0
        lbr4.wm = 2.0  # This cost is on midpoint for joint bounds
        max_iter = 200

        # u_input = np.array([joints.copy() for k in range(T + 1)])  # could be improved by trac-ik
        # u_input[0] = j_0  # joints.copy()#joints_des.copy()
        u_input = j_0
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
                ''

            elif i == T / 2:
                sample_pose[2] = 0.11
            linear_poses.append(sample_pose)

        for i in range(0, T / 2 + 1):
            sample_pose = x0 + (linear_poses[T / 2] - x0) * float(i + 1) / (T / 2)
            sample_pose[3] = x_des[3]
            sample_pose[4] = x_des[4]
            sample_pose[5] = x_des[5]
            linear_poses[i] = sample_pose

        for i in range(T / 2, T):
            j = i - 10
            sample_pose = linear_poses[T / 2] + (x_des - linear_poses[T / 2]) * float(j + 1) / (T / 2)
            sample_pose[3] = x_des[3]
            sample_pose[4] = x_des[4]
            sample_pose[5] = x_des[5]
            linear_poses[i] = sample_pose


        # Interpolate x_desired:
        x_d = linear_poses

        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}

        # Initialize opt classes:
        # cost_fns = cost_fn.CostFns(final_grasp_sdf_cost, final_grasp_f_sdf_cost, final_grasp_sdf_gradient, final_grasp_f_sdf_gradient, lbr4, x_d, u0)
        cost_fns = cost_fn.CostFns(info_gain_cost, final_f_pose_cost, info_gain_cost_gradient, final_f_pose_gradient, lbr4, x_d, u0)
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

        u_new, x_new = opta.optimize(optimizer, u0, x_d, u_input, lbr4)

        x_final = lbr4.end_effector_pose_array(u_new[-1])

        print('Desired:')
        print('x_des', x_des)
        print('x_final', x_final)

        # Interpolate trajectory:
        u_interpol = interpolate_traj(u_new, self.T * 3)

        # if(len(u_new)<self.T):
        #  u_new=interpolate_traj(u_new,self.T+1)

        # print('Final joint angles:')
        # print(u_new)
        # print len(u_interpol)
        return u_interpol, u_new




    def optimize_mov_obj(self, x_d, j_0, angles, x_des2):
        T = self.T
        self.dyn_model = baxterKinematics(T=T)
        self.dyn_model.allegro = baxter_hand_model(T, 60)
        lbr4 = self.dyn_model

        x_des = x_d  # lbr4.end_effector_pose_array(joints_des)
        joints = j_0  # np.random.uniform(lbr4.bounds[0],lbr4.bounds[1])#joints_des.copy()
        lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        lbr4.dimension_wt = np.array([1.0, 1.0, 0.7, 0.3, 0.3, 0.3])

        lbr4.wr = 0.003
        lbr4.wc = 0.0
        lbr4.wf = 100.0
        lbr4.wm = 0.01  # This cost is on midpoint for joint bounds
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
            '''
            sample_pose = x0_2 + (x_des2 - x0_2) * float(i + 1) / T
            sample_pose[3] = x_des2[3]
            sample_pose[4] = x_des2[4]
            sample_pose[5] = x_des2[5]
            sample_pose[6] = x_des2[6]
            '''
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
                #print 'ik_solution', temp_ik
                u_input[i] = temp_ik[:]

            elif i == T/2:
                sample_pose[2] = 0.11
                sample_pose_2 = x0_2 + (x_des2 - x0_2) * float(i + 1) / T
                l_pose = Pose()
                l_pose.position.x = sample_pose_2[0]
                l_pose.position.y = sample_pose_2[1]
                l_pose.position.z = sample_pose[2]
                l_pose.orientation.x = x_des2[3]
                l_pose.orientation.y = x_des2[4]
                l_pose.orientation.z = x_des2[5]
                l_pose.orientation.w = x_des2[6]
                temp_ik = self.ik_solver.ik_solve(l_pose, angles)
                # print 'ik_solution', temp_ik
                u_input[i] = temp_ik[:]
                print u_input[i]
            linear_poses.append(sample_pose)

        for i in range(0,T/2+1):
            sample_pose = x0 + (linear_poses[T/2] - x0) * float(i + 1) / (T/2)
            sample_pose[3] = x_des[3]
            sample_pose[4] = x_des[4]
            sample_pose[5] = x_des[5]
            linear_poses[i] = sample_pose

        for i in range(T/2,T):
            j = i -10
            sample_pose = linear_poses[T/2] + (x_des - linear_poses[T/2]) * float(j + 1) / (T/2)
            sample_pose[3] = x_des[3]
            sample_pose[4] = x_des[4]
            sample_pose[5] = x_des[5]
            linear_poses[i] = sample_pose

        l_pose = Pose()
        l_pose.position.x = x_des2[0]
        l_pose.position.y = x_des2[1]
        l_pose.position.z = x_des2[2]
        l_pose.orientation.x = x_des2[3]
        l_pose.orientation.y = x_des2[4]
        l_pose.orientation.z = x_des2[5]
        l_pose.orientation.w = x_des2[6]
        temp_ik = self.ik_solver.ik_solve(l_pose, angles)
        #print 'ik_solution', temp_ik
        u_input[T] = temp_ik[:]
        temp_u_final = u_input[T]
        temp_u_mid = u_input[T/2]
        print 'temp_u_mid'
        print temp_u_mid
        temp_u_first = u_input[0]

        for i in range(T/2+1):
            sample_input = temp_u_first + (temp_u_mid - temp_u_first) * float(i + 1) / (T/2)
            u_input[i] = sample_input[:]

        for i in range(T/2,T+1):
            j = i - 10
            sample_input = temp_u_mid + (temp_u_final - temp_u_mid) * float(j + 1) / (T/2)
            u_input[i] = sample_input[:]


        # Building transformation matrix:


        x_d = linear_poses

        # u0 = u_input[0]
        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}
        # opt_options = {'MAXIT': 5000, 'IPRINT': 0}

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

        u_new, x_new = opta.optimize(optimizer, u0, x_d, u_input, lbr4)

        x_final = lbr4.end_effector_pose_array(u_new[-1])

        print 'Desired: '
        print 'x_des', x_des
        print 'x_final', x_final

        # Interpolate trajectory:
        u_interpol = interpolate_traj(u_new, self.T * 3)

        #if(len(u_new)<self.T):
        #  u_new=interpolate_traj(u_new,self.T+1)

        print "Final joint angles:"
        print u_new
        # print len(u_interpol)
        return u_interpol, u_new


    def get_ik_plan(self,x_d,j_0,angles,x_des2):
        
        arm_j0=j_0[0:7]
        hand_j0=j_0[7:]
        #u_new,u_ik=self.optimize_ik(x_d,arm_j0,angles,x_des2)



        u_new, u_ik = self.optimize_mov_obj(x_d, arm_j0, angles, x_des2)
        # return u_ik
        u_new_info, u_ik_info = self.optimize_info_gain_one(x_d, u_ik, angles, x_des2)
        # u_new_info, u_ik_info = self.optimize_mov_obj_info_gain(x_d, u_ik, angles, x_des2)



        #print arm_j0
        #send_traj(u_interp,'planner/ik_traj')

        #u_new,u_t=self.optimize_final_grasp_sdf(x_d,u_ik,hand_j0)
        #return u_new
        return u_ik_info



