# This program computes open loop trajectory for performing in grasp manipulation.
#from cost_ik_lbr4 import *
#from cost_ik_lbr4_self_collision import *

import geometry_msgs.msg
import tf2_ros
from cost_palm_collision import *
from cost_final_grasp import *
from cost_final_grasp_sdf import *
from cost_ik_baxter_grasp import *
import traj_opt as tjO
import time
import sys
import pickle
import traj_opt as tjO
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
from kdl_baxter_entropy_plot import *
from kdl_baxter_hand_model import *
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
import  pyOpt_cost_function as cost_fn
from interpolate_traj import *

sys.path.insert(0,path_opt_pkg+'/scripts')
#from trajectory_viz import *


# path_collision_pkg=rp.get_path('pcl_mesh_tools')+'/python'
# sys.path.insert(0,path_collision_pkg)
#from convex_collision_checker import *
# from convex_collision_checker_client import *


from ik_plan import ik_solver
#delta_t=1.0/60 
#T=10

import rospy
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
import matplotlib.pyplot as plt

class baxter_ik:
    def __init__(self,T,delta_t=1.0/60 ):
        self.dyn_model=baxterKinematics(T=T)
        self.dyn_model.allegro=baxter_hand_model(T,60)
        self.T=T
        self.delta_t=delta_t
        self.ik_solver = ik_solver()

    def compute_entropy_vis_tra(self,joint_array):
        self.dyn_model_2 = baxterKinematics_entropy_plot(T=self.T)
        self.dyn_model_2.init_transform_pointcloud_kdl()
        self.dyn_model_2.init_gpflow_gpy(0.1)
        x_axis = np.arange(0., len(joint_array), 1)
        y_axis = np.zeros(len(joint_array))
        for i in range(len(joint_array)):
            joint = joint_array[i]
            cloud_out = self.dyn_model_2.FK_pointcloud(joint)
            cloud_center = self.dyn_model_2.FK_pointcloud_center(joint)
            after_ray,distance = self.dyn_model_2.call_ray_cast(cloud_out,cloud_center)
            var = self.dyn_model_2.gpy.predict(after_ray, full_cov=True)
            var = var[1]
            temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
            info_gain = np.linalg.det(temp_var)
            info_gain_after = np.log(info_gain) * (0.5)
            y_axis[i] = info_gain_after

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xlabel('time_step')
        ax.set_ylabel('conditional_entropy')
        ax.plot(x_axis, y_axis, 'r-')
        plt.show()




    def optimize_full(self,x_d,u_ik,hand_j0):
        T=self.T
        self.dyn_model=baxterKinematics(T=T)
        self.dyn_model.allegro=baxter_hand_model(T,60)
        lbr4=self.dyn_model

        '''
        temporily comment
        '''
        lbr4.c_checker=convex_collision_checker_client()

        x_des=x_d
        #joints=np.random.uniform(lbr4.bounds[0],lbr4.bounds[1])#joints_des.copy()
        #lbr4.dimension=np.array([10.0,10.0,10.0,0.0,0.0,0.0])
        lbr4.position_dimension=np.array([1.0,1.0,1.0,0.0,0.0,0.0])
        lbr4.dimension_wt=np.array([1.0,1.0,1.0,0.1,0.1,0.1])
    
        lbr4.wr=0.01
        lbr4.wc=100.0
        lbr4.wsafe=1.0
        lbr4.wf=30.0
        lbr4.wm=0.01 # This cost is on midpoint for joint bounds
        lbr4.d_safe=0.0004# safe limit for collision checking
    
        max_iter=10
    
   

        u_input=u_ik#np.array([joints.copy() for k in range(T+1)])         
       
        u0=u_input[0]
        x0=lbr4.end_effector_pose_array(u_input[0])
        # Interpolate x_desired:
        linear_poses=[]
        for i in range(T):
            sample_pose=x0+(x_des-x0)*float(i+1)/T
            # Building transformation matrix:
            linear_poses.append(sample_pose)


        x_d=linear_poses

    
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}
        
        # Initialize opt classes:
        cost_fns=cost_fn.CostFns(cost,final_cost,cost_gradient,final_cost_gradient,lbr4,x_d,u0)
        constraint_fns=constraint_fn.ConstraintFns(lbr4,u0,0.2)
        # Bounds for States and input:
        u_bounds=lbr4.bounds

        # compute mid points for joint positions:
        lbr4.joint_mid_pts=(lbr4.bounds[1]+lbr4.bounds[0])/2.0

        lbr4.allegro.preshape=hand_j0
        # Adding bounds for x:
        
        low_bounds=[]
        up_bounds=[]
        for i in range(T):
            low_bounds.extend(u_bounds[0])
            up_bounds.extend(u_bounds[1])
    
        variable_bounds=[low_bounds,up_bounds]
        
        opta=tjO.TrajOpt(lbr4,cost_fns,constraint_fns,variable_bounds,"SNOPT",opt_options)

    
        optimizer=opta.optimizer_init()  
        #allegro_model=dyn_model.fkModel(robot_model)
        u_new,x_new=opta.optimize(optimizer,u0,x_d,u_input,lbr4)


        x_final=lbr4.end_effector_pose_array(u_new[-1])



        print 'Desired: '
        print x_des
        print x_final
        

        lbr4.c_checker.collision_checker.close()

        # Interpolate trajectory:
        u_interpol=interpolate_traj(u_new,T*3)
        print "Final joint angles:"
        print u_new[-1]
        print 'u_interpolate',u_interpol
        return u_interpol,u_new

    def optimize_ik(self,x_d,j_0,angles,x_des2):
        T=2
        self.dyn_model=baxterKinematics(T=T)
        self.dyn_model.allegro=baxter_hand_model(T,60)
        lbr4=self.dyn_model
        
    
        x_des=x_d#lbr4.end_effector_pose_array(joints_des)
        joints=j_0#np.random.uniform(lbr4.bounds[0],lbr4.bounds[1])#joints_des.copy()
        lbr4.position_dimension=np.array([1.0,1.0,1.0,0.0,0.0,0.0])
        lbr4.dimension_wt=np.array([1.0,1.0,1.0,0.1,0.1,0.1])
    
    
        lbr4.wr=0.1
        lbr4.wc=0.0
        lbr4.wf=200.0
        lbr4.wm=0.01 # This cost is on midpoint for joint bounds
        max_iter=5000
    
   

        u_input=np.array([joints.copy() for k in range(T+1)])  #could be improved by trac-ik
        u_input[0]=j_0#joints.copy()#joints_des.copy()
        u0=u_input[0]
        x0=lbr4.end_effector_pose_array(u_input[0])

        #x_des_2 = np.append(np.array(x_des), np.zeros(1))
        x_des_2 = x_des2
        x0_2 = lbr4.end_effector_pose_array(u_input[0],get_quaternion=True)

        # Interpolate x_desired:
        linear_poses=[]
        for i in range(T):
            sample_pose=x0+(x_des-x0)*float(i+1)/T
            if i == (T-1):
                sample_pose_2 = x0_2 + (x_des2 - x0_2) * float(i + 1) / T
                l_pose = Pose()
                l_pose.position.x = sample_pose_2[0]
                l_pose.position.y = sample_pose_2[1]
                l_pose.position.z = sample_pose_2[2]
                l_pose.orientation.x = sample_pose_2[3]
                l_pose.orientation.y = sample_pose_2[4]
                l_pose.orientation.z = sample_pose_2[5]
                l_pose.orientation.w = sample_pose_2[6]
                temp_ik = self.ik_solver.ik_solve(l_pose, angles)
                print 'ik_solution',temp_ik
                u_input[i] = temp_ik[:]
            linear_poses.append(sample_pose)
        u_input[T] = temp_ik[:]
        temp_u_final = u_input[T]
        temp_u_first = u_input[0]

        for i in range(T):
            sample_input=temp_u_first+(temp_u_final-temp_u_first)*float(i+1)/T
            #print 'sample_input', sample_input
            u_input[i] = sample_input[:]

        # Building transformation matrix:


        x_d=linear_poses

        #u0 = u_input[0]
        opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0}

        # Initialize opt classes:
        cost_fns=cost_fn.CostFns(ik_cost,ik_final_cost,ik_cost_gradient,ik_final_cost_gradient,lbr4,x_d,u0)
        constraint_fns=constraint_fn.ConstraintFns(lbr4,u0,0.2)
        # Bounds for States and input:
        u_bounds=lbr4.bounds

        # compute mid points for joint positions:
        lbr4.joint_mid_pts=(lbr4.bounds[1]+lbr4.bounds[0])/2.0
    
        # Adding bounds for x:
 
        low_bounds=[]
        up_bounds=[]
        for i in range(T):
            low_bounds.extend(u_bounds[0])
            up_bounds.extend(u_bounds[1])
    
        variable_bounds=[low_bounds,up_bounds]
        
        opta=tjO.TrajOpt(lbr4,cost_fns,constraint_fns,variable_bounds,"SNOPT",opt_options)

    
        optimizer=opta.optimizer_init()  

        u_new,x_new=opta.optimize(optimizer,u0,x_d,u_input,lbr4)


        x_final=lbr4.end_effector_pose_array(u_new[-1])

        print 'Desired: '
        print 'x_des',x_des
        print 'x_final',x_final

    
        # Interpolate trajectory:
        u_interpol=interpolate_traj(u_new,self.T*10)

        #if(len(u_new)<self.T):
           #u_new=interpolate_traj(u_new,self.T+1)
           
        print "Final joint angles:"
        print u_new
        #print len(u_interpol)
        return u_interpol,u_new

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

    def optimize_mov_obj(self, x_d, j_0, angles, x_des2):
        T = 20
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
        '''
        br = tf2_ros.StaticTransformBroadcaster()
        tf_send = []
        for i in range(0, len(linear_poses)):
            print 'ii', i
            temp_pose = np.array(linear_poses[i])
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base"
            t.child_frame_id = "static_gripper_" + str(i)
            t.transform.translation.x = temp_pose[0]
            t.transform.translation.y = temp_pose[1]
            t.transform.translation.z = temp_pose[2]
            t.transform.rotation.x = temp_pose[3]
            t.transform.rotation.y = temp_pose[4]
            t.transform.rotation.z = temp_pose[5]
            t.transform.rotation.w = temp_pose[6]
            tf_send.append(t)

        print 'publish the posearray'
        br.sendTransform(tf_send)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # cloud_pub.publish(obj_resp.obj.cloud)
            rate.sleep()
        '''
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

        '''
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # cloud_pub.publish(obj_resp.obj.cloud)
            rate.sleep()

        
        for i in range(T):
            sample_input = temp_u_first + (temp_u_final - temp_u_first) * float(i + 1) / T
            if i <= T/2:
                u_input[i] = sample_input[:]
            else:
                u_input[i] = sample_input[:]
            # print 'sample_input', sample_input
        '''

        # Building transformation matrix:


        x_d = linear_poses

        # u0 = u_input[0]
        opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}
        #opt_options = {'MAXIT': 5000, 'IPRINT': 0}

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
        #opta = tjO.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SLSQP", opt_options)

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

    def optimize_mov_obj_2(self, x_d, j_0, angles, x_des2):
        T = self.T
        # self.dyn_model = baxterKinematics(T=T)
        # self.dyn_model.allegro = baxter_hand_model(T, 60)
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

    def get_ik_plan(self,x_d,j_0,angles,x_des2):
        
        arm_j0=j_0[0:7]
        hand_j0=j_0[7:]
        #u_new,u_ik=self.optimize_ik(x_d,arm_j0,angles,x_des2)
        #u_new, u_ik = self.optimize_mov_obj_2(x_d, arm_j0, angles, x_des2)
        isValid, u_ik, fstr = self.optimize_mov_obj_2(x_d, arm_j0, angles, x_des2)
        #print arm_j0
        #send_traj(u_interp,'planner/ik_traj')

        #u_new,u_t=self.optimize_final_grasp_sdf(x_d,u_ik,hand_j0)
        #return u_new
        return u_ik



