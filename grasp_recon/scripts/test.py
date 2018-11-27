#!/usr/bin/env python
import rospy
from optimization_baxter.srv import *
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import sys
import numpy as np
from rospkg import RosPack
import PyKDL
import baxter_interface
from baxter_interface import CHECK_VERSION
#joint trajectory call for baxter
from joint_trajectory import Trajectory
rp=RosPack()
rp.list()

#path=rp.get_path('ll4ma_kdl')+'/scripts'

#sys.path.insert(0,path)
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
from kdl_baxter import *
#from kdl_lbr4_model import *
# from kdl_lbr4_allegro_model import *
joints_0=np.append(np.array([0.0,0.2,0.0,-1.0,0.0,0.0,1.5]),np.zeros(10))

dyn_model=baxterKinematics(T=10)

#allegro = lbr4_allegro_model(10, 60)
lbr4=dyn_model
lbr4.position_dimension=np.array([1.0,1.0,1.0,0.0,0.0,0.0])
lbr4.dimension_wt=np.array([1.0,1.0,1.0,0.1,0.1,0.1])
lbr4.wr=0.1
lbr4.wc=0.0
lbr4.wf=100.0
lbr4.wm=0.01
path_opt_pkg=rp.get_path('optimization_baxter')
u_bounds=lbr4.bounds
lbr4.joint_mid_pts=(lbr4.bounds[1]+lbr4.bounds[0])/2.0
#sys.path.insert(0,path_opt_pkg+'/src')
sys.path.insert(0,path_opt_pkg+'/src/ik_planner')


# def ik_cost_gradient(x_des, u, dyn_model, t=-1):
#     wr = dyn_model.wr
#     wm = dyn_model.wm
#     gradient_fk = np.zeros(dyn_model.m)
#     diff_pose = (x_des - dyn_model.end_effector_pose_array(u)) * dyn_model.dimension_wt
#     jacobian = dyn_model.jacobian_full(u)
#     print('m',np.matrix(jacobian[3:, 4:]))
#     # Gradient: [du dx]
#     gradient_fk[0:dyn_model.m] = -1.0 * wr * np.array(np.matrix(diff_pose * dyn_model.position_dimension) *
#                                                       np.matrix(jacobian)).ravel()
#
#     gradient_fk[4:dyn_model.m] += -1.0 * wr * np.array(np.matrix(diff_pose[3:]) * np.matrix(jacobian[3:, 4:])).ravel()
#
#     # l_mid=wm*np.sum(np.abs(dyn_model.joint_mid_pts-u))
#     gradient_fk += -2.0 * wm * (dyn_model.joint_mid_pts - u)
#
#     return gradient_fk
import  pyOpt_cost_function as cost_fn
#import ik_cost,ik_final_cost,ik_cost_gradient,ik_final_cost_gradient from the file below
from cost_ik_baxter_grasp import *
import pyOpt_lbr4_vel_constraint as constraint_fn
import traj_opt as tjO

T = 10
arm_j0=joints_0[0:7]
#hand_j0=joints_0[7:]
u_input=np.array([arm_j0.copy() for k in range(T+1)])
u_input[0] = joints_0[0:7]  # joints.copy()#joints_des.copy()
u0 = u_input[0]
x0=lbr4.end_effector_pose_array(arm_j0)


linear_poses=[]
x_des=np.zeros(6)
x_des = lbr4.end_effector_pose_array([0.11, -0.62, 1.15, 1.32, -0.80, 1.27, -2.39])
for i in range(T):
    sample_pose=x0+(x_des-x0)*float(i+1)/T
    # Building transformation matrix:
    linear_poses.append(sample_pose)

temp_grad = ik_cost_gradient(linear_poses[0],u0,dyn_model)
print('grad',temp_grad)

test_array = [[1,2,4,4,4,5],[1,3,3,3,3,3]]
n = 400
test_array = np.dot(test_array,1.0/n)
print(test_array)
import  pyOpt_cost_function as cost_fn
opt_options={'MAXIT':50,'IPRINT':0}
#opt_options={'Iterations limit':5,'Solution':'No','iPrint':0,'iSumm':0}
        # Initialize opt classes:
cost_fns=cost_fn.CostFns(ik_cost,ik_final_cost,ik_cost_gradient,ik_final_cost_gradient,lbr4,linear_poses,arm_j0)
constraint_fns=constraint_fn.ConstraintFns(lbr4,arm_j0,0.2)
u_bounds=lbr4.bounds
lbr4.joint_mid_pts=(lbr4.bounds[1]+lbr4.bounds[0])/2.0

#print joint_mid_pts
low_bounds = []
up_bounds = []
for i in range(T):
    low_bounds.extend(u_bounds[0])
    up_bounds.extend(u_bounds[1])

variable_bounds=[low_bounds,up_bounds]
opta=tjO.TrajOpt(lbr4,cost_fns,constraint_fns,variable_bounds,"SLSQP",opt_options)

optimizer=opta.optimizer_init()
x_d=linear_poses
u_new,x_new=opta.optimize(optimizer,u0,x_d,u_input,lbr4)

x_final=lbr4.end_effector_pose_array(u_new[-1])

from interpolate_traj import *
u_interpol=interpolate_traj(u_new,T*10)
if(len(u_new)<T):
    u_new=interpolate_traj(u_new,T+1)

# finish optimize_ik above

u_ik = u_new


x_n=lbr4.predict_T(x0,u_input,10)
x_d=x_des
q0 = optimizer.cost_fns.get_q(x_n,u_input)

#f=optimizer.cost_fns.objectiveFunction(q0)
x,u = optimizer.cost_fns.get_x_u_from_q(q0)
x_d=optimizer.cost_fns.x_d
u_arr = u[1:-1]
x_d_arr = x_d[:-1]
t=np.array([range(lbr4.T-1)]).T
data_arr=np.concatenate((x_d_arr,u_arr),axis=1)
data_arr=list(np.concatenate((data_arr,t),axis=1))
optimizer.cost_fns.multi_cost(data_arr[0])
cost_arr=[optimizer.cost_fns.multi_cost(k) for k in data_arr]
cost=sum(cost_arr)
cost =optimizer.cost_fns.final_cost(x_d[-1],u[-1],lbr4)
gradient_arr=[optimizer.cost_fns.multi_gradient(data) for data in data_arr]
gradient=np.ravel(gradient_arr)
gradient=np.append(gradient,optimizer.cost_fns.gradient_u_final(x_d[-1],u[-1],lbr4))


optimizer.objfunc(arm_j0)

u_new,x_new=opta.optimize(optimizer,arm_j0,x_des,u_input,lbr4)

x_final=lbr4.end_effector_pose_array(u_new[-1])

print x_final


if __name__ == '__main__':
    print 'hello'
    limb = 'left'
    print limb
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    # print("Getting robot state... ")
    # rs = baxter_interface.RobotEnable(CHECK_VERSION)
    # print("Enabling robot... ")
    # rs.enable()
    # print("Running. Ctrl-c to quit")
    # positions = {
    #     'left': [-0.11, -0.62, -1.15, 1.32, 0.80, 1.27, 2.39],
    #     'right': [0.11, -0.62, 1.15, 1.32, -0.80, 1.27, -2.39],
    # }
    #
    # traj = Trajectory(limb)
    # rospy.on_shutdown(traj.stop)
    # # Command Current Joint Positions first
    # limb_interface = baxter_interface.limb.Limb(limb)
    # current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # current_names = [joint for joint in limb_interface.joint_names()]
    # print current_names
    # print current_angles
    # traj.add_point(current_angles, 0.0)
    # p1 = positions[limb]
    #
    # # optimization for trajectory
    # joints_0 = np.array(current_angles)
    #
    # dyn_model = baxterKinematics(T=10)
    #
    # # allegro = lbr4_allegro_model(10, 60)
    # lbr4 = dyn_model
    # lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    # lbr4.dimension_wt = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
    # lbr4.wr = 0.1
    # lbr4.wc = 0.0
    # lbr4.wf = 100.0
    # lbr4.wm = 0.01
    # T = 10
    # arm_j0 = joints_0[0:7]
    # # hand_j0=joints_0[7:]
    # u_input = np.array([arm_j0.copy() for k in range(T + 1)])
    # u_input[0] = joints_0[0:7]  # joints.copy()#joints_des.copy()
    # u0 = u_input[0]
    # x0 = lbr4.end_effector_pose_array(arm_j0)
    #
    # linear_poses = []
    # x_des = np.array(lbr4.end_effector_pose_array([x * 1.25 for x in p1]))
    # print 'x_des'
    # print x_des
    # for i in range(T):
    #     sample_pose = x0 + (x_des - x0) * float(i + 1) / T
    #     # Building transformation matrix:
    #     linear_poses.append(sample_pose)
    #
    # #opt_options = {'MAXIT': 5000, 'IPRINT': 0}
    # opt_options={'Iterations limit':500,'Solution':'No','iPrint':0,'iSumm':0}
    # # Initialize opt classes:
    # cost_fns = cost_fn.CostFns(ik_cost, ik_final_cost, ik_cost_gradient, ik_final_cost_gradient, lbr4, linear_poses,
    #                            arm_j0)
    # constraint_fns = constraint_fn.ConstraintFns(lbr4, arm_j0, 0.2)
    # u_bounds = lbr4.bounds
    # lbr4.joint_mid_pts = (lbr4.bounds[1] + lbr4.bounds[0]) / 2.0
    #
    # # print joint_mid_pts
    # low_bounds = []
    # up_bounds = []
    # for i in range(T):
    #     low_bounds.extend(u_bounds[0])
    #     up_bounds.extend(u_bounds[1])
    #
    # variable_bounds = [low_bounds, up_bounds]
    # opta = tjO.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SNOPT", opt_options)
    #
    # optimizer = opta.optimizer_init()
    # x_d = linear_poses
    # u_new, x_new = opta.optimize(optimizer, u0, x_d, u_input, lbr4)
    #
    # x_final = lbr4.end_effector_pose_array(u_new[-1])
    #
    # from interpolate_traj import *
    #
    # u_interpol = interpolate_traj(u_new, T * 10)
    # if (len(u_new) < T):
    #     u_new = interpolate_traj(u_new, T + 1)
    #
    # # finish optimize_ik above
    #
    # u_ik = u_new
    # #print u_ik
    # #print 'u_interpol'
    # #print u_interpol
    # #print 'final'
    # print [x * 1.25 for x in p1]
    # i = 0.5
    # for x in u_ik:
    #     traj.add_point(x,i)
    #     #print x
    #     i +=2
    # traj.start()
    # traj.wait(30.0)
    # print("Exiting - Joint Trajectory Action Test Complete")
    # #rospy.spin()
