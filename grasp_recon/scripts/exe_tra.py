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
from geometry_msgs.msg import Pose,PoseArray,PoseStamped,Point
#joint trajectory call for baxter
from joint_trajectory import Trajectory

from baxter_interface import (
    Gripper
)
# from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_smoothing.srv import GetSmoothTraj
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
    rospy.init_node('exe_tra')
    # traj_set_pub = rospy.Publisher('traj_set', MarkerArray, queue_size=1)
    # rate = rospy.Rate(1)
    # x = 1
    # delta_x = 0.1
    #
    # obj_m_array = MarkerArray()
    # for i in range(1):
    #     s = np.random.uniform(0.1, 1, 3)
    #     obj_m = Marker()
    #     obj_m.header.frame_id = 'base'
    #     obj_m.id = i
    #     obj_m.type = Marker.LINE_STRIP
    #     obj_m.action = Marker.ADD
    #     obj_m.color.a = 1
    #     obj_m.color.r = s[0]
    #     obj_m.color.g = s[1]
    #     obj_m.color.b = s[2]
    #     obj_m.scale.x = 0.005
    #     point_t = Point()
    #     point_t.x = x
    #     point_t.y = 1
    #     point_t.z = 1
    #     obj_m.points.append(point_t)
    #     obj_m_array.markers.append(obj_m)
    #     for q in range(10):
    #         x = x + delta_x
    #         point_t = Point()
    #         point_t.x = x
    #         point_t.y = 1
    #         point_t.z = 1
    #         obj_m_array.markers[i].points.append(point_t)
    #
    #
    #
    #
    # while not rospy.is_shutdown():
    #     traj_set_pub.publish(obj_m_array)
    #     rate.sleep()

    joint_names = _J_NAMES = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']

    j_wp = []
    j_traj = JointTrajectory()

    u_final = np.load('save_best_traj.npy')

    print(u_final)

    n = len(u_final)

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    limb_interface = baxter_interface.limb.Limb('left')
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    current_angles = np.array(current_angles)

    j_wp.append(current_angles)
    for i in range(n):
        j_wp.append(u_final[i])

    # # store in joint trajectory msg:
    for i in range(n+1):
        pt = JointTrajectoryPoint()
        pt.positions = j_wp[i]
        j_traj.points.append(pt)

    j_traj.joint_names = joint_names

    max_acc = np.ones(7) * 20
    max_vel = np.ones(7) * 20
    # call service for smoothing:
    rospy.wait_for_service('/get_smooth_trajectory')
    traj_call = rospy.ServiceProxy('/get_smooth_trajectory', GetSmoothTraj)
    resp = traj_call(j_traj, max_acc, max_vel, 0.01, 0.001)
    u_final_list = []
    for j in range(len(resp.smooth_traj.points)):
        temp = resp.smooth_traj.points[j].positions
        temp = np.array(temp)
        u_final_list.append(temp)

    print(u_final_list)

    limb = 'left'
    traj = Trajectory(limb)
    i = 0.1
    for x in u_final_list:
        traj.add_point(x,i)
        i +=0.014
    traj.start()
    traj.wait(130.0)
    print("Exiting - Joint Trajectory One Action Test Complete")

    # left = baxter_interface.Gripper('left', CHECK_VERSION)
    # print("Opening left gripper... ")
    # counter = 0
    # while counter != 10 and not rospy.is_shutdown():
    #     left.open()
    #     counter += 1
    #     rospy.sleep(0.2)
    # print("Opening left gripper finish")

    '''
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    '''

    '''
    lift off baxter's arm
    '''
    limb = 'left'
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    current_names = [joint for joint in limb_interface.joint_names()]

    joints_0 = np.append(np.array(current_angles), np.zeros(2))
    '''
    hard coded the gripper postion, need to revise later
    '''
    joints_0[-1] = -0.020833
    joints_0[-2] = -0.020833
    print
    'joint_0', joints_0

    from sensor_msgs.msg import JointState
    joint_angle = JointState()
    angles = baxter_interface.Limb(limb).joint_angles()

    j = 0
    for i in angles:
        joint_angle.name.append(i)
        joint_angle.position.append(current_angles[j])
        j += 1

    x_des = np.zeros(6)
    x_des_2 = np.zeros(7)

    #goal postion in joint space
    goal_angle = [-0.6952767921090638, -0.5303738574113818, 0.12118448224294769, 1.3805827090968723,
                  -0.16490293469768197, 0.7179030087303736, 0.14802914603094242]

    rp = RosPack()
    path_opt_pkg = rp.get_path('optimization_baxter')
    sys.path.insert(0, path_opt_pkg + '/src/ik_planner')
    # from compute_ik_lbr4_grasp import lbr4_ik
    from compute_ik_baxter_lift_off import baxter_ik_lift_off

    ik_obj = baxter_ik_lift_off(10)
    temp_pose = ik_obj.dyn_model.end_effector_pose_array(np.array(current_angles), get_quaternion=True)
    x_des_2[0] = temp_pose[0]
    x_des_2[1] = temp_pose[1]
    x_des_2[2] = temp_pose[2] + 0.116
    x_des_2[3] = temp_pose[3]
    x_des_2[4] = temp_pose[4]
    x_des_2[5] = temp_pose[5]
    x_des_2[6] = temp_pose[6]

    temp_pose_2 = ik_obj.dyn_model.end_effector_pose_array(np.array(current_angles))
    goal_des = ik_obj.dyn_model.end_effector_pose_array(np.array(goal_angle))

    import math

    distance_diff = goal_des -  temp_pose_2

    dis_diff = math.sqrt(distance_diff[0]*distance_diff[0] + distance_diff[1]*distance_diff[1] + distance_diff[2]*distance_diff[2])



    x_des[0] = temp_pose_2[0]
    x_des[1] = temp_pose_2[1]
    x_des[2] = temp_pose_2[2] + 0.116
    x_des[3] = temp_pose_2[3]
    x_des[4] = temp_pose_2[4]
    x_des[5] = temp_pose_2[5]

    u_final = ik_obj.get_ik_plan(x_des, joints_0, joint_angle, x_des_2)

    i = 0.5
    traj2 = Trajectory(limb)
    traj2.add_point(current_angles, 0.0)
    for x in u_final:
        traj2.add_point(x, i)
        i += 0.5
    traj2.start()
    traj2.wait(60.0)

    print("distance difference is:",dis_diff)

    print("Exiting - Full Joint Trajectory Action Test Complete")


