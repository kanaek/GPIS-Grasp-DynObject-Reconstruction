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
from compute_ik_baxter_mov_obj import baxter_ik
from ik_plan import ik_solver
import tf2_ros
import tf
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from joint_trajectory import Trajectory
from vis_robot import VizInHand


baxter_urdf_path=rp.get_path('baxter_description')
baxter_urdf_path = baxter_urdf_path+'/urdf/baxter2_correct_2.urdf'
'''
l_gripper_r_finger_joint:lower="-0.020833" upper="0.0"
l_gripper_l_finger_joint:lower="0.0" upper="0.020833"
'''
def get_joint_angle():
    limb = 'left'

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    current_names = [joint for joint in limb_interface.joint_names()]
    ik_obj = baxter_ik(10)
    temp_pose = ik_obj.dyn_model.end_effector_pose_array(np.array(current_angles))
    print 'temp_pose'
    print 'temp_pose'
    print temp_pose
    return current_angles


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
    traj = Trajectory(limb)
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
    traj.add_point(current_angles, 0.0)

    joint_angle = JointState()
    angles = baxter_interface.Limb(limb).joint_angles()
    #seed angle for initial guess
    real_angles = [-0.46057773156259546, -0.4314320965927726, 0.18714565612202047, 0.9771457618830085, -0.16988837225830958, 0.6964272776999778, 0.016490293469768196]
    j = 0
    for i in angles:
        joint_angle.name.append(i)
        joint_angle.position.append(real_angles[j])
        j+=1

    '''
    use ransac_segmentation to get the pose
    '''
    ik_obj = baxter_ik(20)
    #goal_angle = [-0.9123350735948498, -0.44600491407768406, 0.22357769983429907, 1.1888351106111956, -0.3762087882288977, 0.6078398871995953, 0.279951493789088]
    goal_angle = [-1.0438739261560241, -0.6642136811543842, 0.4118738415472336, 1.584218658688661, -0.5345923045780667, 0.7221214558970586, 0.5150340495325276]
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
    # rotate = 0
    # for k_i in range((len(u_final))):
    #     u_final[k_i][6] = u_final[k_i][6] + rotate
    #     rotate -= 0.157
    #
    # np.save('save_best_traj_rot', u_final)
    np.save('save_move_only', u_final)

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
    #[-0.2548184   0.21439318 -1.2718522   1.95518267  2.38949217  1.42388705
    #3.0399397 ]

    i = 2

    # for x in u_final:
    #     traj.add_point(x,i)
    #     #print x
    #     i +=2
    # traj.start()
    # traj.wait(30.0)

    path = '/home/kanrun/catkin_ws/src/baxter_common/baxter_description/'
    urdf_path = path + '/urdf/baxter2_correct_2.urdf'
    v_hand = VizInHand(path, urdf_path)
    pose_arr = []
    for i in range(0, len(u_final)):
         if i == 0 or i == len(u_final)-1 or i == len(u_final)/2:
            temp_u = u_final[i]
            poses = []
            for j in range(7):
                current = temp_u[0:j + 1]
                temp = v_hand.FK_joint(current, j)
                poses.append(temp)
            pose_arr.append(poses)

    obj_pub, m_arr = v_hand.viz_robot_traj(pose_arr)
    # ik_obj.compute_entropy_vis_tra(u_final)


    print("Exiting - Joint Trajectory Action Test Complete")

    end = time.time()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # obj_pub.publish(m_arr)
        #cloud_pub.publish(obj_resp.obj.cloud)
        rate.sleep()



if __name__=='__main__':
    rospy.init_node("grasp_client")
    # Update collision server


    '''
    # Call collision initialization service:
    rospy.wait_for_service('collision_checker/update_env_cloud')
    update_env_cloud=rospy.ServiceProxy('collision_checker/update_env_cloud',update_env)
    res=update_env_cloud()
    
    print res.init_env
    #Wait for grasp services:
    rospy.wait_for_service('collision_checker')
    #rospy.wait_for_service('grasp/contact_checker')

    print "All services online"
    '''
    #current_angles = get_joint_angle()
    print 'current_angles'
    #print current_angles
    # Get plan
    j_traj=get_plan()

    '''
    needs to call trajectory server on the baxter
    '''
    # Send joint trajectory to visualizer
