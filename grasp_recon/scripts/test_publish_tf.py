#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
#import moveit_commander
import rospy
import tf2_ros
#from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
#import moveit_msgs.msg
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
limb_interface = baxter_interface.limb.Limb(limb)
current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
current_names = [joint for joint in limb_interface.joint_names()]
left = baxter_interface.Gripper(limb, CHECK_VERSION)
gripper_current = left.position()
joints_0 = np.array(current_angles)
print 'joint_0',joints_0

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
# path_collision_pkg=rp.get_path('pcl_mesh_tools')+'/python'
# sys.path.insert(0,path_collision_pkg)
# from convex_collision_checker_client import *
# #from robot_convex_vis import *
# from cost_palm_collision import *
arm_j0=joints_0[0:7]
hand_j0=joints_0[7:]
lbr4.allegro.preshape=hand_j0
linear_poses = []

temp_pose = lbr4.end_effector_pose_array(joints_0,True)
#temp_pose_2 = lbr4.allegro.hand_link_pose_array(joints_0,1,1)
print 'temp_pose'
print temp_pose
#you have to start baxter moveit launch file in order to run this program
if __name__ == '__main__':
    #rospy.init_node('dynamic_tf_broadcaster')
    br = tf2_ros.StaticTransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base"
    t.child_frame_id = "static_camera1"
    t.transform.translation.x = temp_pose[0]
    t.transform.translation.y = temp_pose[1]
    t.transform.translation.z = temp_pose[2]
    t.transform.rotation.x = temp_pose[3]
    t.transform.rotation.y = temp_pose[4]
    t.transform.rotation.z = temp_pose[5]
    t.transform.rotation.w = temp_pose[6]
    #br.sendTransform(t)

    temp = []
    temp.append(t)
    '''
    temp = []
    for i in range(0,2):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        frame_id = "static_gripper" + str(i)
        print frame_id
        t.header.frame_id = "base"
        t.child_frame_id = frame_id
        t.transform.translation.z = 1
        t.transform.rotation.w = 1
        temp.append(t)
        print i
    '''
    br.sendTransform(temp)
    rospy.spin()