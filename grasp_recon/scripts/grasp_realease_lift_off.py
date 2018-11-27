#!/usr/bin/env python
# This server listens to a desired end-effector pose and computes a joint trajectory:
import rospy
from optimization_baxter.srv import *
from pcl_mesh_tools.srv import update_env
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from tabletop_obj_segmentation.srv import SegmentGraspObject
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose,PoseArray
import sys
import time
import numpy as np
from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('optimization_baxter')+'/src'
import baxter_interface
from baxter_interface import CHECK_VERSION
sys.path.insert(0,path)
from kdl_baxter import *
import geometry_msgs.msg

#from trajectory_pub import *
path_opt_pkg=rp.get_path('optimization_baxter')
sys.path.insert(0,path_opt_pkg+'/src/ik_planner')
#from compute_ik_lbr4_grasp import lbr4_ik
from compute_ik_baxter_lift_off import baxter_ik_lift_off
from ik_plan import ik_solver
import tf2_ros
import tf
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from joint_trajectory import Trajectory
import PyKDL
from vis_robot import VizInHand
from trajectory_pub import trajectoryServer
baxter_urdf_path=rp.get_path('baxter_description')
baxter_urdf_path = baxter_urdf_path+'/urdf/baxter2_correct_2.urdf'
'''
l_gripper_r_finger_joint:lower="-0.020833" upper="0.0"
l_gripper_l_finger_joint:lower="0.0" upper="0.020833"
'''

def get_plan():
    limb = 'left'
    '''
    joints need to be adapt to baxter
    '''
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    current_names = [joint for joint in limb_interface.joint_names()]


    '''
    automatically open the left gripper
    '''
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    print("Opening left gripper... ")
    counter = 0
    while counter != 10 and not rospy.is_shutdown():
        left.open()
        counter += 1
        rospy.sleep(0.2)
    print("Opening left gripper finish")

    joints_0 = np.append(np.array(current_angles), np.zeros(2))
    '''
    hard coded the gripper postion, need to revise later
    '''
    joints_0[-1] = -0.020833
    joints_0[-2] = -0.020833
    print 'joint_0',joints_0


    joint_angle = JointState()
    angles = baxter_interface.Limb(limb).joint_angles()

    j = 0
    for i in angles:
        joint_angle.name.append(i)
        joint_angle.position.append(current_angles[j])
        j+=1

    '''
    # ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    # '''
    x_des=np.zeros(6)
    x_des_2 = np.zeros(7)
    '''
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    '''

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
    x_des[0]=temp_pose_2[0]
    x_des[1]=temp_pose_2[1]
    x_des[2]=temp_pose_2[2] + 0.116
    x_des[3] = temp_pose_2[3]
    x_des[4] = temp_pose_2[4]
    x_des[5] = temp_pose_2[5]

    u_final = ik_obj.get_ik_plan(x_des, joints_0,joint_angle,x_des_2)

    br = tf2_ros.StaticTransformBroadcaster()
    tf_send = []
    for i in range(0, len(u_final)):
        print 'ii', i
        temp_u = np.array(u_final[i])
        temp_pose = ik_obj.dyn_model.end_effector_pose_array(temp_u, get_quaternion=True)
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
    path = '/home/kanrun/catkin_ws/src/baxter_common/baxter_description/'
    urdf_path = path + '/urdf/baxter2_vis.urdf'
    v_hand = VizInHand(path, urdf_path)

    pose_arr = []
    u_final[:,-1] = u_final[-1,-1]
    u_final_vis = np.array([u_final[-1]])
    traj_server = trajectoryServer(10)
    u_final_list = list(u_final)
    for j in range(0, len(u_final_list)):
            temp = u_final_list[j]
            temp = np.append(temp, 0.020833)
            temp = np.append(temp, -0.020833)
            u_final_list[j] = temp
    pub_tr, disp_tr = traj_server.send_traj(u_final_list)

    # for j in range(0, len(u_final_vis)):
    #         temp_u = u_final_vis[j]
    #         temp_u = np.append(temp_u,-0.020833)
    #         temp_u = np.append(temp_u, -0.020833)
    #         poses = []
    #         for i in range(11):
    #             if i <= 6:
    #                 current = temp_u[0:i + 1]
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #             elif i <= 8:
    #                 current = temp_u[0:8]
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #             else:
    #                 current = temp_u[0:7]
    #                 current = np.append(current,temp_u[8])
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #         pose_arr.append(poses)
    #
    # obj_pub, m_arr = v_hand.viz_robot_traj(pose_arr)

    i = 2
    traj = Trajectory(limb)
    traj.add_point(current_angles, 0.0)
    for x in u_final:
        traj.add_point(x,i)
        i +=2
    traj.start()
    traj.wait(60.0)

    print("Exiting - Joint Trajectory Action Test Complete")

    print("Closing left gripper... ")
    counter = 0
    while counter != 10 and not rospy.is_shutdown():
        left.close()
        counter += 1
        rospy.sleep(0.2)
    print("Closing left gripper finish")

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     # obj_pub.publish(m_arr)
    #     pub_tr.publish(disp_tr)
    #     rate.sleep()


if __name__=='__main__':
    rospy.init_node("grasp_client")
    # Update collision server


    # Call collision initialization service:
    # rospy.wait_for_service('collision_checker/update_env_cloud')
    # update_env_cloud=rospy.ServiceProxy('collision_checker/update_env_cloud',update_env)
    # res=update_env_cloud()
    #
    # print res.init_env
    # #Wait for grasp services:
    # rospy.wait_for_service('collision_checker')
    #rospy.wait_for_service('grasp/contact_checker')

    print "All services online"


    # Get plan
    j_traj=get_plan()

    '''
    needs to call trajectory server on the baxter
    '''
    # Send joint trajectory to visualizer
    '''
    t_name='planner/ik_traj'
    traj_server=trajectoryServer(100,topic_name=t_name,init_node=False)
    raw_input('Send joint traj?')
    traj_server.viz_joint_traj(j_traj)

    
    # Send joint trajectory to robot
    t_name='lbr4_allegro/follow_joint_trajectory'
    robot_traj_server=trajectoryServer(100,topic_name=t_name,init_node=False,init_jtas_client=True) 
    send_=int(raw_input('send to robot? 1- yes, 0-no'))
    if(send_==1):
        robot_traj_server.joint_traj_jtas(j_traj)   
    '''
