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
import  pyOpt_cost_function as cost_fn
import pyOpt_cost_function_info_gain as cost_fn_info_gain
from kdl_baxter import *
from kdl_baxter_info_gain import *
#from kdl_lbr4_model import *
#from kdl_lbr4_allegro_model import *
from kdl_baxter_hand_model import *
#joints_0 = np.array([-0.44830588525951215, -0.09779127522769512, 0.1928980840765908, 1.5757817643552914, -0.8199127311247536, 0.2646116859102339, -2.9855101084219866])
joints_0=np.append(np.array([0.0,0.2,0.0,-1.0,0.0,0.0,1.5]),np.zeros(1))
dyn_model=baxterKinematics(T=10)
dyn_model = baxterKinematics_info_gain(T=10)
# dyn_model=baxterKinematics_info_gain(T=10)
#dyn_model=lbr4Kinematics(T=10)
allegro = baxter_hand_model(10, 60)
#allegro = lbr4_allegro_model(10, 60)
lbr4=dyn_model
lbr4.allegro = allegro
lbr4.position_dimension=np.array([1.0,1.0,1.0,0.0,0.0,0.0])
lbr4.dimension_wt=np.array([1.0,1.0,1.0,0.1,0.1,0.1])
lbr4.wr=0.1
lbr4.wc=0.0
lbr4.wf=100.0
lbr4.wm=0.01
lbr4.d_safe = 0.00
path_opt_pkg=rp.get_path('optimization_baxter')
#sys.path.insert(0,path_opt_pkg+'/src')
sys.path.insert(0,path_opt_pkg+'/src/ik_planner')
import pyOpt_lbr4_vel_constraint as constraint_fn
import traj_opt as tjO
# path_collision_pkg=rp.get_path('pcl_mesh_tools')+'/python'
# sys.path.insert(0,path_collision_pkg)
# from convex_collision_checker_client import *
from cost_info_gain import *
from cost_palm_collision import *
arm_j0=joints_0[0:7]
hand_j0=joints_0[7:]
lbr4.allegro.preshape=hand_j0
linear_poses = []
T = 10
max_iter=50
u_input=np.array([arm_j0.copy() for k in range(T+1)])

#lbr4.c_checker = convex_collision_checker_client()
u0 = u_input[0]
x0 = lbr4.end_effector_pose_array(u_input[0])
#print x0
# Interpolate x_desired:
p1 = [0.11, -0.62, 1.15, 1.32, -0.80, 1.27, -2.39]
x_des = np.array(lbr4.end_effector_pose_array([x * 1.25 for x in p1]))
for i in range(T):
    sample_pose = x0 + (x_des - x0) * float(i + 1) / T
    # Building transformation matrix:
    linear_poses.append(sample_pose)

x_d = linear_poses
print 'x_des',x_des
print 'x0',x0
print 'linear_pose',linear_poses
opt_options = {'Iterations limit': max_iter, 'Solution': 'Nopr', 'iPrint': 0, 'iSumm': 0}
cost_fns=cost_fn.CostFns(cost,final_cost,cost_gradient,final_cost_gradient,lbr4,x_d,u0)
# cost_fns = cost_fn_info_gain.CostFns_info(info_gain_cost, final_f_pose_cost, info_gain_cost_gradient, final_f_pose_gradient, lbr4, x_d, u0)
constraint_fns=constraint_fn.ConstraintFns(lbr4,u0,0.2)
# Bounds for States and input:
u_bounds=lbr4.bounds

# compute mid points for joint positions:
lbr4.joint_mid_pts=(lbr4.bounds[1]+lbr4.bounds[0])/2.0

lbr4.allegro.preshape=hand_j0
low_bounds = []
up_bounds = []
for i in range(T):
    low_bounds.extend(u_bounds[0])
    up_bounds.extend(u_bounds[1])

variable_bounds = [low_bounds, up_bounds]

opta = tjO.TrajOpt(lbr4, cost_fns, constraint_fns, variable_bounds, "SNOPT", opt_options)
optimizer=opta.optimizer_init()
x_n=lbr4.predict_T(x0,u_input,10)
x_d=x_des
q0 = optimizer.cost_fns.get_q(x_n,u_input)
x,u = optimizer.cost_fns.get_x_u_from_q(q0)
print(x,u)
# optimizer.objfunc(q0)

'''
x_d=optimizer.cost_fns.x_d
u_arr = u[1:-1]
print 'hello'
print u_arr
print 'hello'
print u[-1]
print 'hello'
x_d_arr = x_d[:-1]
t=np.array([range(lbr4.T-1)]).T
data_arr=np.concatenate((x_d_arr,u_arr),axis=1)
data_arr=list(np.concatenate((data_arr,t),axis=1))

#l_col = optimizer.cost_fns.multi_cost(data_arr[0])
l_col = optimizer.cost_fns.multi_gradient(data_arr[0])
#l_col = optimizer.cost_fns.final_cost(x_d[-1],u_arr[-1],optimizer.cost_fns.robot)
l_col = optimizer.cost_fns.gradient_u_final(x_d[-1],u_arr[-1],optimizer.cost_fns.robot)
#print l_col
'''
if __name__ == '__main__':
    rospy.init_node("rsdk_joint_trajectory_client")

    '''
    print 'hello'
    limb = 'right'
    print limb
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    positions = {
        'left': [-0.11, -0.62, -1.15, 1.32, 0.80, 1.27, 2.39],
        'right': [0.11, -0.62, 1.15, 1.32, -0.80, 1.27, -2.39],
    }
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    current_names = [joint for joint in limb_interface.joint_names()]
    print current_names
    print current_angles
    traj.add_point(current_angles, 0.0)
    p1 = positions[limb]

    joints_0 = np.append(np.array(current_angles),np.zeros(1))
    dyn_model = baxterKinematics(T=10)
    allegro = baxter_hand_model(10, 60)
    # allegro = lbr4_allegro_model(10, 60)
    lbr4 = dyn_model
    lbr4.allegro = allegro
    lbr4.position_dimension = np.array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    lbr4.dimension_wt = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
    lbr4.wr = 0.1
    lbr4.wc = 0.0
    lbr4.wf = 100.0
    lbr4.wm = 0.01
    lbr4.c_checker = convex_collision_checker_client()
    lbr4.d_safe = 0.00
    T = 10
    arm_j0 = joints_0[0:7]
    hand_j0=joints_0[7:]
    u_input = np.array([arm_j0.copy() for k in range(T + 1)])
    u_input[0] = joints_0[0:7]  # joints.copy()#joints_des.copy()
    u0 = u_input[0]
    x0 = lbr4.end_effector_pose_array(arm_j0)
    linear_poses = []
    x_des = np.array(lbr4.end_effector_pose_array([x * 1.25 for x in p1]))
    print 'x_des'
    print x_des
    for i in range(T):
        sample_pose = x0 + (x_des - x0) * float(i + 1) / T
        # Building transformation matrix:
        linear_poses.append(sample_pose)
    opt_options = {'Iterations limit': max_iter, 'Solution': 'No', 'iPrint': 0, 'iSumm': 0}
    cost_fns = cost_fn.CostFns(cost, final_cost, cost_gradient, final_cost_gradient, lbr4, x_d, u0)
    constraint_fns = constraint_fn.ConstraintFns(lbr4, u0, 0.2)
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
    
    #r_poses=lbr4.allegro.collision_link_poses([-0.44830588525951215])
    #print r_poses
    #print x_final
    #rospy.spin()
    '''
