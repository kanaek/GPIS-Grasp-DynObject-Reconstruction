#!/usr/bin/env python
# This program computes open loop trajectory for performing in grasp manipulation.
#from cost_ik_lbr4 import *
#from cost_ik_lbr4_self_collision import *

import geometry_msgs.msg
import tf2_ros
import tf
# from cost_palm_collision import *
from cost_final_grasp import *
from cost_table_collision_as_contraint import *
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
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trajectory_smoothing.srv import GetSmoothTraj
from collision_check.srv import collision_checker_msg
import traj_opt_collision as tjO_collsion
import pyOpt_cost_function_collision as cost_fn_collsion

class baxter_ik_prm:
    def __init__(self,T,delta_t=1.0/60 ):
        self.dyn_model=baxterKinematics(T=T)
        self.dyn_model.allegro=baxter_hand_model(T,delta_t)
        self.T=T
        self.delta_t=delta_t
        self.ik_solver = ik_solver()
        self.test_cloud = None

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

        opt_prob.addObj('f')
        u_bounds = lbr4.bounds
        j_0[:,6] = j_0[0,6]
        j_0[9][6] = j_0[9][6]+ 2.2
        print('j_0',j_0[9])
        opt_prob.addVarGroup('u',len(j_0[0]), 'c', lower=u_bounds[0], upper=u_bounds[1], value=j_0[9])
        # opt_prob.addCon('g', 'i',upper=-0.01)

        method=getattr(pyOpt,"SNOPT")
        optimizer=method(options=opt_options)

        [fstr,xstr,inform]=optimizer(opt_prob,sens_type=grad_obj_func) #sens_type could be finite differences. sens_type='FD'
        xstr = np.array(xstr)
        print(xstr)

        return [xstr],[xstr]

    def prm_info_gain(self, x_d, j_0):
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

        def overall_info_gain_cost(u_list,dyn_model):
            overall = []
            for i in range(len(u_list)):
                u = u_list[i]
                cloud_center = dyn_model.FK_pointcloud_center(u)
                cloud_out_1 = dyn_model.FK_pointcloud(u)
                after_ray, distance = dyn_model.call_ray_cast_return_num(cloud_out_1, cloud_center)
                overall.extend(after_ray)
                print(i)

            overall = list(set(overall))
            after_ray = dyn_model.append_pointcloud(overall)
            print(overall,len(overall))
            var = dyn_model.gpy.predict(after_ray, full_cov=True)
            var = var[1]

            temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
            info_gain = np.linalg.det(temp_var)
            info_gain = np.log(info_gain) * 0.5

            return info_gain

        def sample(ubound_low,ubound_high,u):
            u_final = []
            return u_final


        def info_gain_cost(u, dyn_model):

            info_gain = 0
            before = rospy.get_time()
            cloud_center = dyn_model.FK_pointcloud_center(u)
            wm = dyn_model.wm

            cloud_out_1 = dyn_model.FK_pointcloud(u)

            after_ray,distance = dyn_model.call_ray_cast(cloud_out_1, cloud_center,return_index=False)

            if after_ray.shape[0]==0:
                print('out of scope in function')
                return 0,distance

            # var = dyn_model.gpflow.predict_f_full_cov(after_ray)
            var = dyn_model.gpy.predict(after_ray, full_cov=True)
            var = var[1]

            temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
            info_gain = np.linalg.det(temp_var)
            info_gain = np.log(info_gain) * 0.5

            after = rospy.get_time()
            print("Current time ", (after - before))

            # print('distance', distance)
            return info_gain * (wm),distance

        u_bounds = lbr4.bounds

        # info_gain,distance = info_gain_cost(j_0[-1],lbr4)
        # print('info_gain one',info_gain)
        # # j_0[:,6] = j_0[0,6]
        # j_0[-1][6] = j_0[-1][6]+ 3.0
        # print('j_0',j_0[-1])
        # info_gain, distance = info_gain_cost(j_0[-1], lbr4)
        # print('info_gain one after', info_gain)

        overall_info_cost =overall_info_gain_cost(j_0,lbr4)
        print('overall_info_cost after', overall_info_cost)


        # opt_prob.addVarGroup('u',len(j_0[0]), 'c', lower=u_bounds[0], upper=u_bounds[1], value=j_0[9])

        return j_0


    '''
    evaluate the cost for different trajectories
    '''
    def gmm_train_traj(self,x_d,u_ik,criteria):
        self.dyn_model_2 = baxterKinematics_info_gain(T=1)
        self.dyn_model_2.init_transform_pointcloud_kdl()
        self.dyn_model_2.init_gpflow_gpy()
        lbr4 = self.dyn_model_2

        def compute_elite_set_cost(dyn_model,u_list,x_d):
            overall = []
            for i in range(len(u_list)):
                u = u_list[i]
                cloud_center = dyn_model.FK_pointcloud_center(u)
                cloud_out_ = dyn_model.FK_pointcloud_speed(u)
                # cloud_out_1 = dyn_model.FK_pointcloud(u)
                after_ray, distance = dyn_model.call_ray_cast_return_num(cloud_out_, cloud_center)
                overall.extend(after_ray)

            overall = list(set(overall))
            after_ray = dyn_model.append_pointcloud(overall)
            var = dyn_model.gpy.predict(after_ray, full_cov=True)
            var = var[1]

            temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
            info_gain = np.linalg.det(temp_var)
            info_gain = np.log(info_gain) * 0.5

            return info_gain,after_ray


        def violate_constraint(dyn_model,traj):
            lower_bound = dyn_model.bounds[0]
            higher_bound = dyn_model.bounds[1]
            traj = traj.reshape(-1,7)
            # print('traj',traj)
            for i in range(7):
                traj_temp = traj[:, i]
                temp_result = traj_temp[(traj_temp > lower_bound[i]) & (traj_temp < higher_bound[i])]
                if len(temp_result) == len(traj):
                    # print('no violate')
                    None
                else:
                    return True

            '''
            do collision checking without final control
            '''
            traj_without_init_end = np.array(traj[1:-1])
            for i in range(len(traj_without_init_end)):
                temp_u = traj_without_init_end[i]
                cloud_center = dyn_model.FK_pointcloud_center(temp_u)
                point_center = Point()
                point_center.x = cloud_center[0][0]
                point_center.y = cloud_center[0][1]
                point_center.z = cloud_center[0][2]
                result = self.dyn_model_2.collision_table(temp_u, point_center)
                # print('collision',i)
                # print('res',result.success,result.obj_success)
                if result.success==True or result.cone_distance>-0.03:
                    return True


            return False


        def generate_sample_zcon(dyn_model,covariance_list,mean_list,weight_list,num):
            i = 0
            result_list = []
            choice = np.arange(len(weight_list))
            result = np.random.choice(choice, p=weight_list)
            while i < num:
            # for i in range(num):
            #     choice = np.arange(len(weight_list))
            #     result = np.random.choice(choice, p=weight_list)
                '''
                63 needs to be changed depending on the size of the control input list* dimenstion of the joint space !!!!!
                '''
                normal_result = np.random.normal(0, 1, 63)
                co_result = np.dot(covariance_list[result], normal_result)
                # co_result = covariance_list[result] * normal_result
                co_result = mean_list[result] + co_result
                if violate_constraint(dyn_model,co_result):
                    print('violate constr!!!',i)
                    continue
                else:
                    co_result = co_result.reshape(-1,7)
                    # print('no violate',co_result)
                    print('no violate',i)
                    result_list.append(co_result)
                    i = i + 1
                    if i <= num:
                        continue
                    else:
                        return result_list


            return result_list




        from sklearn import mixture

        # max_cost = 0
        # max_input = []
        # max_pointcloud = []
        '''
        cross-entropy randomized motion planning
        '''
        '''
        q-th quantile of H(Z)
        '''
        quantile = 0.1
        u_ik = np.array(u_ik)
        # u_ik_new = []
        # for counter in range(len(u_ik)):
        #     temp_ik = u_ik[counter]
        #     temp_ik = temp_ik.ravel()
        #     u_ik_new.append(temp_ik)
        #
        # u_ik_new = np.array(u_ik_new)


        overall_max_cost = 0
        overall_max_traj = None
        overall_max_pointcloud = None
        '''
        generate elite set for the 
        '''
        counter = 0
        after_ray_all = []
        cost_all = []
        for sample in u_ik:
            temp_cost, after_ray = compute_elite_set_cost(lbr4, sample, x_d)
            counter += 1
            cost_all.append(temp_cost)
            after_ray_all.append(after_ray)
            print('temp_cost:', temp_cost, counter)

        '''
        keep the maxium cost and corresponding control list
        '''
        cost_all = np.array(cost_all)
        after_ray_all = np.array(after_ray_all)
        max_index = np.argmax(cost_all)
        # return u_ik[max_index], after_ray_all[max_index]
        max_cost = cost_all[max_index]
        max_input = u_ik[max_index]
        max_pointcloud = after_ray_all[max_index]

        quantile_result = len(cost_all) * quantile
        quantile_result = int(quantile_result) + 1
        a = cost_all[np.argsort(-cost_all)]
        print('a', a)
        itemindex = np.argwhere(cost_all >= a[quantile_result - 1])
        # cost_result = cost_all[itemindex[:, 0]]
        print('first iter')
        sample_list = np.array(u_ik)
        sample_list_nex_iter = sample_list[itemindex[:, 0]]
        u_ik_new = []
        for ik in range(len(sample_list_nex_iter)):
            temp_ik = sample_list_nex_iter[ik]
            temp_ik = temp_ik.ravel()
            u_ik_new.append(temp_ik)

        u_ik_new = np.array(u_ik_new)

        '''
        fit the GMM with different number of components, needs to be tuned
        '''
        gmm = mixture.GaussianMixture(n_components=1, covariance_type='full')
        gmm.fit(u_ik_new)



        '''
        for the first iteration, the elite set does not need to be built
        '''
        x = np.identity(int(gmm.covariances_[0].shape[0]))
        '''
        the noise to prevent degeneracy needs to be tuned
        '''
        for i in range(int(gmm.covariances_[0].shape[0])):
            x[i,i] =  np.random.normal(0, 0.01, 1)
        gmm.covariances_[:] = gmm.covariances_[:] + x

        while True:
            '''
            generate N samples for the next iteration
            '''
            N = 20
            sample_list = generate_sample_zcon(lbr4, gmm.covariances_, gmm.means_, gmm.weights_, N)
            counter = 0
            after_ray_all = []
            cost_all = []
            for sample in sample_list:
                temp_cost, after_ray = compute_elite_set_cost(lbr4, sample,x_d)
                counter += 1
                cost_all.append(temp_cost)
                after_ray_all.append(after_ray)
                print('temp_cost:', temp_cost, counter)

            '''
            keep the maxium cost and corresponding control list
            '''
            cost_all = np.array(cost_all)
            after_ray_all = np.array(after_ray_all)
            max_index = np.argmax(cost_all)
            max_cost = cost_all[max_index]
            max_input = sample_list[max_index]
            max_pointcloud = after_ray_all[max_index]
            # if max_cost>overall_max_cost:
            #     overall_max_cost = max_cost
            #     overall_max_traj = max_input
            #     overall_max_pointcloud = max_pointcloud

            quantile_result = len(cost_all) * quantile
            quantile_result = int(quantile_result) + 1
            a = cost_all[np.argsort(-cost_all)]
            print('a',a)
            itemindex = np.argwhere(cost_all >= a[quantile_result - 1])
            cost_result = cost_all[itemindex[:, 0]]
            print('next iter')
            sample_list = np.array(sample_list)
            sample_list_nex_iter = sample_list[itemindex[:, 0]]


            '''
            if the cost of the trajectory doesn't increase anymore , we are going to finish the algorithm
            '''
            # if max(cost_result)>=criteria or max_cost - overall_max_cost <1:
            if max_cost - overall_max_cost < 1:
                if max_cost-overall_max_cost > 0:
                    overall_max_cost = max_cost
                    overall_max_traj = max_input
                    overall_max_pointcloud = max_pointcloud
                print('find the optimal solution, the cost is:', overall_max_cost)
                break
            else:
                if max_cost>overall_max_cost:
                    overall_max_cost = max_cost
                    overall_max_traj = max_input
                    overall_max_pointcloud = max_pointcloud
                u_ik_new = []
                for ik in range(len(sample_list_nex_iter)):
                    temp_ik = sample_list_nex_iter[ik]
                    temp_ik = temp_ik.ravel()
                    u_ik_new.append(temp_ik)

                u_ik_new = np.array(u_ik_new)
                gmm.fit(u_ik_new)
                gmm.covariances_[:] = gmm.covariances_[:] + x
                continue

        return overall_max_traj,max_pointcloud


    '''
    get optimizaiton for different trajectories
    '''
    def optimize_final_grasp_sdf(self, x_d, u_ik, hand_j0):
        print('self.t',self.T)
        T = self.T
        self.dyn_model = baxterKinematics(T=T)
        self.dyn_model.allegro = baxter_hand_model(T, T)
        lbr4 = self.dyn_model

        rospy.wait_for_service('/collision_checker')
        lbr4.collision_checking = rospy.ServiceProxy('/collision_checker',collision_checker_msg)

        x_des = x_d
        # joints=np.random.uniform(lbr4.bounds[0],lbr4.bounds[1])#joints_des.copy()
        # lbr4.dimension=np.array([10.0,10.0,10.0,0.0,0.0,0.0])
        lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        lbr4.dimension_wt = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        lbr4.wr = 0.01
        lbr4.wc = 0.5
        lbr4.wm = 0.3

        lbr4.wf = 0.03
        lbr4.w_cf = 0.3

        lbr4.d_safe = -0.01  # safe limit for collision checking

        max_iter = 500

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
        cost_fns = cost_fn_collsion.CostFns(cost, final_cost, cost_gradient, final_cost_gradient, lbr4, x_d, u0)
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

        opta = tjO_collsion.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SNOPT", opt_options)

        optimizer = opta.optimizer_init()
        # allegro_model=dyn_model.fkModel(robot_model)
        u_new, x_new = opta.optimize(optimizer, u0, x_d, u_input, lbr4)

        # x_final = lbr4.end_effector_pose_array(u_new[-1])

        # print 'Desired: '
        # print x_des
        # print x_final

        # lbr4.c_checker.collision_checker.close()

        # Interpolate trajectory:
        # u_interpol = interpolate_traj(u_new, T * 3)
        # print "Final joint angles:"
        # print u_new[-1]
        # print 'u_interpolate', u_interpol
        return u_new, u_new


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
        # x_des_2 = x_des2
        # x0_2 = lbr4.end_effector_pose_array(u_input[0], get_quaternion=True)

        # Interpolate x_desired:
        linear_poses = []
        for i in range(T):

            sample_pose = x0 + (x_des - x0) * float(i + 1) / T

            # if i == (T - 1):
            #     sample_pose_2 = x0_2 + (x_des2 - x0_2) * float(i + 1) / T
            #     l_pose = Pose()
            #     l_pose.position.x = sample_pose_2[0]
            #     l_pose.position.y = sample_pose_2[1]
            #     l_pose.position.z = sample_pose_2[2]
            #     l_pose.orientation.x = x_des2[3]
            #     l_pose.orientation.y = x_des2[4]
            #     l_pose.orientation.z = x_des2[5]
            #     l_pose.orientation.w = x_des2[6]
            #     temp_ik,isValid = self.ik_solver.ik_solve(l_pose, angles)
            #     #print 'ik_solution', temp_ik
            #     u_input[i] = temp_ik[:]

            if i == T/2:
                sample_pose[2] = 0.11
                sample_pose_2 = x0 + (x_des - x0) * float(i + 1) / T
                l_pose = Pose()
                l_pose.position.x = sample_pose_2[0]
                l_pose.position.y = sample_pose_2[1]
                l_pose.position.z = sample_pose[2]
                l_pose.orientation.x = x_des2[3]
                l_pose.orientation.y = x_des2[4]
                l_pose.orientation.z = x_des2[5]
                l_pose.orientation.w = x_des2[6]
                temp_ik, isValid = self.ik_solver.ik_solve(l_pose, angles)
                u_input[i] = temp_ik[:]
            linear_poses.append(sample_pose)

        for i in range(0,T/2+1):
            sample_pose = x0 + (linear_poses[T/2] - x0) * float(i + 1) / (T/2)
            sample_pose[3] = x_des[3]
            sample_pose[4] = x_des[4]
            sample_pose[5] = x_des[5]
            linear_poses[i] = sample_pose

        for i in range(T/2,T):
            j = i -T/2
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
        temp_ik,isValid = self.ik_solver.ik_solve(l_pose, angles)

        u_input[T] = temp_ik[:]
        temp_u_final = u_input[T]
        temp_u_mid = u_input[T/2]
        temp_u_first = u_input[0]

        for i in range(T/2+1):
            sample_input = temp_u_first + (temp_u_mid - temp_u_first) * float(i + 1) / (T/2)
            u_input[i] = sample_input[:]

        for i in range(T/2,T+1):
            j = i - T/2
            sample_input = temp_u_mid + (temp_u_final - temp_u_mid) * float(j + 1) / (T/2)
            u_input[i] = sample_input[:]


        # Building transformation matrix:


        x_d = linear_poses

        # u0 = u_input[0]
        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}

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

        optimizer = opta.optimizer_init()

        u_new, x_new, fstr = opta.optimize(optimizer, u0, x_d, u_input, lbr4)


        # Interpolate trajectory:
        # u_interpol = interpolate_traj(u_new, self.T * 3)

        return isValid, u_new, fstr



    '''
    compute the baseline for running the algorithm and after running the algorithm
    '''
    def compute_elite_set_cost(self,dyn_model,u_list):
            overall = []
            for i in range(len(u_list)):
                u = u_list[i]
                cloud_center = dyn_model.FK_pointcloud_center(u)
                cloud_out_ = dyn_model.FK_pointcloud_speed(u)
                after_ray, distance = dyn_model.call_ray_cast_return_num(cloud_out_, cloud_center)
                overall.extend(after_ray)

            overall = list(set(overall))
            after_ray = dyn_model.append_pointcloud(overall)
            var = dyn_model.gpy.predict(after_ray, full_cov=True)
            var = var[1]

            temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
            info_gain = np.linalg.det(temp_var)
            info_gain = np.log(info_gain) * 0.5
            return info_gain,after_ray

    def get_ik_plan(self,x_d,j_0,angles,x_des2):
        
        arm_j0=j_0[0:7]
        hand_j0 = j_0[7:]
        self.dyn_model_2 = baxterKinematics_info_gain(T=1)
        # x_d_temp = np.copy(x_d)
        # x_des2_temp = np.copy(x_des2)
        # sigma = 0.7
        # for jk in range(6):
        #    s = np.random.normal(x_d_temp[jk], sigma, 1)
        #    if abs(s) <= 3.14 and jk >= 3:
        #        x_d_temp[jk] = s
        # quat = tf.transformations.quaternion_from_euler(x_d_temp[3], x_d_temp[4], x_d_temp[5])
        # x_des2_temp[3:] = quat
        # isValid, u_ik, fstr = self.optimize_mov_obj(x_d_temp, arm_j0, angles, x_des2_temp)
        # lower_bound = self.dyn_model.bounds[0]
        # higher_bound = self.dyn_model.bounds[1]
        # u_new = []
        # cov = np.identity(7)
        # for i in range(7):
        #     cov[i, i] = 0.003
        #     if i == 6:
        #         cov[i, i] = 0.08
        # for tt in u_ik:
        #     mean = tt
        #     noise_counter = 0
        #     while noise_counter<=1:
        #         x = np.random.multivariate_normal(mean, cov, 1)
        #         result = self.dyn_model_2.collision_table(x[0])
        #         if result.success==True:
        #             print('violate collsion!!!!', noise_counter)
        #             continue
        #         else:
        #
        #             if (x[0] >= lower_bound).all() and (x[0] < higher_bound).all():
        #                 u_new.append(x[0])
        #                 # print('success')
        #                 break
        #             else:
        #                 print('violate constriant!!!!', noise_counter)
        #                 noise_counter += 1
        #                 continue
        #     if noise_counter==2:
        #         print('push original!!!!', noise_counter)
        #         u_new.append(tt)
        # # u_new, u_t = self.optimize_final_grasp_sdf(x_d, u_ik, hand_j0)
        # after_ray = []

        lower_bound = self.dyn_model.bounds[0]
        # lower_bound[-1] = -3
        higher_bound = self.dyn_model.bounds[1]
        # higher_bound[-1] = 3

        '''
        generate random poses for trajectories
        '''
        count = 0
        u_train = []
        num_training = 30


        '''
        define the covariance for the gaussian noise
        '''

        cov = np.identity(7)
        for i in range(7):
            cov[i, i] = 0.015
            if i == 6:
                cov[i, i] = 0.4
        noise_max = 1
        collision_max = 10


        while count<num_training:
            x_d_temp = np.copy(x_d)
            x_des2_temp = np.copy(x_des2)
            sigma = 1.1
            for jk in range(6):
                s = np.random.normal(x_d_temp[jk], sigma, 1)
                if abs(s) <= 3.14 and jk >= 3:
                    x_d_temp[jk] = s

            quat = tf.transformations.quaternion_from_euler(x_d_temp[3], x_d_temp[4], x_d_temp[5])
            x_des2_temp[3:] = quat
            isValid, u_ik,fstr = self.optimize_mov_obj(x_d_temp, arm_j0, angles, x_des2_temp)
            if isValid[0] == False:
                print('isValid i=', count, isValid)
                continue
            else:
                print('isValid i=', count, isValid)
                print('i=, and cost=', count, fstr[0])
                if fstr[0]>=6:
                   continue
                else:
                   count += 1
            u_new = []



            for tt in u_ik:
                mean = tt
                noise_counter = 0
                collision_counter = 0
                while noise_counter<=noise_max and collision_counter<=collision_max:
                    x = np.random.multivariate_normal(mean, cov, 1)
                    cloud_out_ = self.dyn_model_2.end_effector_pose_array(x[0])
                    point_center = Point()
                    point_center.x = cloud_out_[0]
                    point_center.y = cloud_out_[1]
                    point_center.z = cloud_out_[2]
                    result = self.dyn_model_2.collision_table(x[0],point_center)
                    if result.success==True or result.cone_distance>-0.02:
                    #if result.cone_distance > -0.02:
                        print('violate collision!!!!', collision_counter,result.cone_distance)
                        collision_counter += 1
                        continue
                    else:
                        if (x[0] >= lower_bound).all() and (x[0] < higher_bound).all():
                            u_new.append(x[0])
                            break
                        else:
                            print('violate constriant!!!!', noise_counter)
                            noise_counter += 1
                            continue

                if noise_counter==noise_max+1 or collision_counter==collision_max+1:
                    print('push original!!!!', collision_counter, noise_counter)
                    u_new.append(tt)
            u_train.append(u_new)


            # u_train.append(u_ik)

        np.save('u_train', u_train)
        '''
        for benchmark purpose
        '''

        # for i  in range(len(u_ik)):
        #     tt = u_ik[i]
        #     tt[-1] = u_ik[0,-1]- 0.45*i
        # print('tt', u_ik)
        # self.dyn_model_2 = baxterKinematics_info_gain(T=1)
        # self.dyn_model_2.init_transform_pointcloud_kdl()
        # self.dyn_model_2.init_gpflow_gpy()
        # lbr4 = self.dyn_model_2
        # info_gain, after_ray = self.compute_elite_set_cost(lbr4,u_ik)
        # print('info_gain',info_gain)
        '''
        for benchmark purpose
        '''


        '''
        for visualization purpose
        '''
        # traj_set_pub = rospy.Publisher('traj_set', MarkerArray, queue_size=1)
        # obj_m_array = MarkerArray()
        # id_counter = 0
        # rospy.wait_for_service('/get_smooth_trajectory')
        # traj_call = rospy.ServiceProxy('/get_smooth_trajectory', GetSmoothTraj)
        # max_acc = np.ones(7) * 20
        # max_vel = np.ones(7) * 20
        # joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        # #
        # #
        # # overall_max_cost = 0
        # # overall_max_traj = None
        # # overall_max_pointcloud = None
        # for u_int in u_train:
        #     # info_gain, after_ray = self.compute_elite_set_cost(lbr4, u_int)
        #     # if info_gain>overall_max_cost:
        #     #     overall_max_cost = info_gain
        #     #     overall_max_traj = u_int
        #     #     overall_max_pointcloud = after_ray
        #     # print('info_gain', info_gain)
        #     s = np.random.uniform(0.1, 1, 3)
        #     obj_m = Marker()
        #     obj_m.header.frame_id = 'base'
        #     obj_m.id = id_counter
        #     obj_m.type = Marker.LINE_STRIP
        #     obj_m.action = Marker.ADD
        #     obj_m.color.a = 1
        #     obj_m.color.r = s[0]
        #     obj_m.color.g = s[1]
        #     obj_m.color.b = s[2]
        #     obj_m.scale.x = 0.005
        #     obj_m_array.markers.append(obj_m)
        #
        #     '''
        #     smooth the trajectory
        #     '''
        #     j_traj = JointTrajectory()
        #     for i in range(len(u_int)):
        #         pt = JointTrajectoryPoint()
        #         pt.positions = u_int[i]
        #         j_traj.points.append(pt)
        #     j_traj.joint_names = joint_names
        #     resp = traj_call(j_traj, max_acc, max_vel, 0.01, 0.001)
        #     print('smooth:',id_counter)
        #     '''
        #     smooth the trajectory
        #     '''
        #
        #     for j in range(len(resp.smooth_traj.points)):
        #         temp = resp.smooth_traj.points[j].positions
        #         temp = np.array(temp)
        #         temp_pose = self.dyn_model.end_effector_pose_array(temp, get_quaternion=True)
        #         point_t = Point()
        #         point_t.x = temp_pose[0]
        #         point_t.y = temp_pose[1]
        #         point_t.z = temp_pose[2]
        #         obj_m_array.markers[id_counter].points.append(point_t)
        #     #
        #     #
        #     # for u_temp in u_int:
        #     #     temp_pose = self.dyn_model.end_effector_pose_array(u_temp,get_quaternion=True)
        #     #     point_t = Point()
        #     #     point_t.x = temp_pose[0]
        #     #     point_t.y = temp_pose[1]
        #     #     point_t.z = temp_pose[2]
        #     #     obj_m_array.markers[id_counter].points.append(point_t)
        #
        #     id_counter = id_counter +1
        #
        # # u_ik_info = overall_max_traj
        # # after_ray = overall_max_pointcloud
        # # print('max info_gain', overall_max_cost)
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     traj_set_pub.publish(obj_m_array)
        #     rate.sleep()
        '''
        for visualization purpose
        '''


        u_ik_info,after_ray = self.gmm_train_traj(x_d, u_train,60)
        np.save('save_best_traj', u_ik_info)
        return u_ik_info, after_ray



