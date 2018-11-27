#!/usr/bin/env python
# This server listens to a desired end-effector pose and computes a joint trajectory:
import rospy
from optimization_baxter.srv import *
# from pcl_mesh_tools.srv import update_env
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
from compute_ik_baxter_mov_obj_info import baxter_ik_info
from ik_plan import ik_solver
import tf2_ros
import tf
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from joint_trajectory import Trajectory
from vis_robot import VizInHand
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2



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
    #joints_0=np.append(np.array([0.0,0.2,0.0,-1.0,0.0,0.0,1.5]),np.zeros(16))
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    # traj = Trajectory(limb)
    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    current_names = [joint for joint in limb_interface.joint_names()]

    left = baxter_interface.Gripper(limb, CHECK_VERSION)
    gripper_current = left.position()

    joints_0 = np.append(np.array(current_angles), np.zeros(2))
    '''
    hard coded the gripper postion, need to revise later
    '''
    joints_0[-1] = -0.020833
    joints_0[-2] = -0.020833
    print 'joint_0',joints_0
    # traj.add_point(current_angles, 0.0)

    joint_angle = JointState()
    angles = baxter_interface.Limb(limb).joint_angles()
    #seed angle for initial guess
    real_angles = [-1.2524953133084402, -0.3601019899561009, -0.04141748127290617, 0.9422476989586154, -0.06557767868210143, 0.9541360500647273, -1.9907235674782957]
    j = 0
    for i in angles:
        joint_angle.name.append(i)
        joint_angle.position.append(current_angles[j])
        j+=1


    ik_obj = baxter_ik_info(20)
    goal_angle = [-1.114437042398753,-0.29912625363765566,0.22357769983429907,0.8156942839580688,-0.3160000423043952,1.0833739314440736,-1.5692623460067783]
    x_des = ik_obj.dyn_model.end_effector_pose_array(np.array(goal_angle))
    x_des=np.array(x_des)

    x_des_2 = ik_obj.dyn_model.end_effector_pose_array(np.array(goal_angle), get_quaternion=True)
    x_des_2 = np.array(x_des_2)


    start = time.time()
    print x_des


    temp_pose = ik_obj.dyn_model.end_effector_pose_array(np.array(current_angles), get_quaternion=True)
    x_des_2[3] = temp_pose[3]
    x_des_2[4] = temp_pose[4]
    x_des_2[5] = temp_pose[5]
    x_des_2[6] = temp_pose[6]
    temp_pose_2 = ik_obj.dyn_model.end_effector_pose_array(np.array(current_angles))
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
    print('ufinal',u_final)
    br.sendTransform(tf_send)

    # temp_u = np.array(u_final[i])


    # cloud_out = ik_obj.dyn_model_2.vis_cloud(u_final[0])
    # HEADER = Header(frame_id='/base')
    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #           PointField('y', 4, PointField.FLOAT32, 1),
    #           PointField('z', 8, PointField.FLOAT32, 1),
    #           PointField('rgb', 16, PointField.FLOAT32, 1)
    #           ]
    # obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out)
    # cloud_pub = rospy.Publisher('gpis_debug_final', PointCloud2, queue_size=10)
    #
    path = '/home/kanrun/catkin_ws/src/baxter_common/baxter_description/'
    urdf_path = path + '/urdf/baxter2_vis.urdf'
    v_hand = VizInHand(path, urdf_path)
    pose_arr = []
    for j in range(0, len(u_final)):
            temp_u = u_final[j]
            temp_u = np.append(temp_u,-0.020833)
            temp_u = np.append(temp_u, -0.020833)
            poses = []
            for i in range(11):
                if i <= 6:
                    current = temp_u[0:i + 1]
                    temp = v_hand.FK_joint(current, i)
                    poses.append(temp)
                elif i <= 8:
                    current = temp_u[0:8]
                    temp = v_hand.FK_joint(current, i)
                    poses.append(temp)
                else:
                    current = temp_u[0:7]
                    current = np.append(current,temp_u[8])
                    temp = v_hand.FK_joint(current, i)
                    poses.append(temp)
            pose_arr.append(poses)
    # pose_arr = []
    # for i in range(0, len(u_final)):
    #      if i == 0 or i == len(u_final)-1 or i == len(u_final)/2:
    #         temp_u = u_final[i]
    #         poses = []
    #         for j in range(7):
    #             current = temp_u[0:j + 1]
    #             temp = v_hand.FK_joint(current, j)
    #             poses.append(temp)
    #         pose_arr.append(poses)

    obj_pub, m_arr = v_hand.viz_robot_traj(pose_arr)
    # cloud_pub, after_gpis = ik_obj.dyn_model_2.vis_cloud(u_final[-1])


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # cloud_pub.publish(after_gpis)
        obj_pub.publish(m_arr)
        rate.sleep()

    '''
    ///////////////
    publish final pointcloud
    '''


    i = 2
    # for x in u_final:
    #     traj.add_point(x,i)
    #     #print x
    #     i +=2
    # traj.start()
    # traj.wait(30.0)

    print("Exiting - Joint Trajectory Action Test Complete")

    end = time.time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cloud_pub.publish(obj_cloud_gpis)
        rate.sleep()



if __name__=='__main__':
    rospy.init_node("grasp_client")
    # Update collision server



    # Call collision initialization service:
    # rospy.wait_for_service('collision_checker/update_env_cloud')
    # update_env_cloud=rospy.ServiceProxy('collision_checker/update_env_cloud',update_env)
    # res=update_env_cloud()
    '''
    print res.init_env
    #Wait for grasp services:
    rospy.wait_for_service('collision_checker')
    #rospy.wait_for_service('grasp/contact_checker')

    print "All services online"
    '''
    #print current_angles
    # Get plan
    j_traj=get_plan()

    '''
    needs to call trajectory server on the baxter
    '''
    # Send joint trajectory to visualizer
