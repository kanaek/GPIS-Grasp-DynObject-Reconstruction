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
from geometry_msgs.msg import Pose,PoseArray,PoseStamped
#joint trajectory call for baxter
from joint_trajectory import Trajectory
rp=RosPack()
rp.list()

#path=rp.get_path('ll4ma_kdl')+'/scripts'

'''
l_gripper_r_finger_joint:lower="-0.020833" upper="0.0"
l_gripper_l_finger_joint:lower="0.0" upper="0.020833"
'''
import baxter_interface
from baxter_interface import CHECK_VERSION

#sys.path.insert(0,path)
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src')
import  pyOpt_cost_function as cost_fn
from kdl_baxter import *
#from kdl_lbr4_model import *
#from kdl_lbr4_allegro_model import *
from kdl_baxter_hand_model import *


rospy.init_node("grasp_client")
limb = 'left'
rs = baxter_interface.RobotEnable(CHECK_VERSION)
print("Enabling robot... ")
rs.enable()
print("Running. Ctrl-c to quit")
# Command Current Joint Positions first
limb_interface = baxter_interface.limb.Limb(limb)
current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
current_names = [joint for joint in limb_interface.joint_names()]
left = baxter_interface.Gripper(limb, CHECK_VERSION)
gripper_current = left.position()
joints_0 = np.array(current_angles)
print 'joint_0',joints_0

#joints_0=np.append(np.array([-0.09510680884889565, -0.30756314797102546, -1.8365584982958116, 1.0783884938834458, -1.7019516841588667, -1.570029336400721, -1.50176719133982]
#),np.zeros(2))

joints_0 = np.append(joints_0,np.zeros(2))

joints_0[-1] = -0.020833
joints_0[-2] = -0.020833

joints_0_left = joints_0[0:8]
joints_0_right = np.append(joints_0[0:7],joints_0[8])
print 'joint_0', joints_0
print 'joint_0_left', joints_0_left
print 'joint_0_right', joints_0_right


dyn_model=baxterKinematics(T=10)
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
lbr4.d_safe = 0.03
path_opt_pkg=rp.get_path('optimization_baxter')
#sys.path.insert(0,path_opt_pkg+'/src')
sys.path.insert(0,path_opt_pkg+'/src/ik_planner')
import pyOpt_lbr4_vel_constraint as constraint_fn
import traj_opt as tjO
path_collision_pkg=rp.get_path('pcl_mesh_tools')+'/python'
sys.path.insert(0,path_collision_pkg)
from convex_collision_checker_client import *

from cost_palm_collision import *
arm_j0=joints_0[0:7]
hand_j0=joints_0[7:]
lbr4.allegro.preshape=hand_j0
linear_poses = []

T = 10
max_iter=50
u_input=np.array([arm_j0.copy() for k in range(T+1)])

rospy.init_node("grasp_client")

print "All services online"
rospy.wait_for_service('collision_checker/update_env_cloud')
update_env_cloud = rospy.ServiceProxy('collision_checker/update_env_cloud', update_env)
res = update_env_cloud()

print 'res.init_env',res.init_env
# Wait for grasp services:
rospy.wait_for_service('collision_checker')


lbr4.c_checker = convex_collision_checker_client()
r_poses = lbr4.allegro.collision_link_poses(joints_0)

print('r_poses',r_poses)
c_data=dyn_model.c_checker.get_signed_distance(r_poses)
#
#
l_col = 0.0
w_c = 1.0
temp_u = joints_0
for i in range(0, len(c_data)):
    if i ==0:
        l_num = 0
        f_index = 0
    elif i == 1:
        l_num = 1
        f_index = 0
    elif i == 2:
        l_num = 0
        f_index = 1
    elif i==3:
        l_num = 1
        f_index = 1
    link_J = dyn_model.allegro.link_jacobian(temp_u, l_num, f_index)[0:3, :]
    j_len = len(np.ravel(link_J[0, :]))
    print(j_len)
    if (c_data[i]):  # If not empty

        for j in range(len(c_data[i])):
            # Penetration depth (-ve if not in collision)
            p_d = c_data[i][j][3]
            if (p_d > 0.0):  # If objects are in collision:
                l_col += w_c * 0.5 * p_d ** 2
                print
                'in final>0'
            elif (p_d != 0 and -p_d <= dyn_model.d_safe):  # If object is within d_safe but not in collision
                l_col += w_c * 0.5 * np.abs(-p_d - dyn_model.d_safe) ** 2
                print
                'in final=0'
            else:
                l_col += 0.0