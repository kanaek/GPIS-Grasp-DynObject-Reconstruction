#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
# import required allegro model files
from rospkg import RosPack
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import baxter_interface

from baxter_interface import CHECK_VERSION
import tf
rp = RosPack()
rp.list()
import sys
import PyKDL
from geometry_msgs.msg import Pose

path_collision_pkg=rp.get_path('collision_check')
sys.path.insert(0,path_collision_pkg)

from collision_check.srv import collsion_check
from collision_check.srv import cloud_bounding
from nav_msgs.msg import Path

path=rp.get_path('optimization_baxter')+'/src'
sys.path.insert(0,path)
from kdl_baxter_info_gain_one import *

path_ = rp.get_path('optimization_baxter') + '/scripts'
sys.path.insert(0, path_)
# from trajectory_pub import *
import time
'''
l_gripper_r_finger_joint:lower="-0.020833" upper="0.0"
l_gripper_l_finger_joint:lower="0.0" upper="0.020833"
-3.14159265359
'''

BAXTER_LINK_NAMES=['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm',
             'left_wrist','left_gripper_base']
_ALLEGRO_COLLISION_LINK_NAMES = ['l_gripper_l_finger','l_gripper_l_finger_tip','l_gripper_r_finger','l_gripper_r_finger_tip']
class visTrainingTraj:
    def __init__(self,mesh_loc, urdf_file_name, init_node=False, base_link="base"):

        self.mesh_loc = mesh_loc
        self.arm_links = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm',
             'left_wrist','left_gripper_base','l_gripper_l_finger','l_gripper_l_finger_tip','l_gripper_r_finger','l_gripper_r_finger_tip']

        self.arm_meshes = ['S0.ply','S1.ply','E0.ply','E1.ply','W0.ply','W1.ply','W2.ply','electric_gripper_base.ply',
                           'extended_narrow.ply','half_round_tip.ply','extended_narrow.ply','half_round_tip.ply']
        # initialize node
        if (init_node):
            rospy.init_node('marker_viz')
        # initialize publisher
        self.obj_pub = rospy.Publisher('arm_marker', MarkerArray, queue_size=1)

        urdf_file = file(urdf_file_name, 'r')
        self.robot = Robot.from_xml_string(urdf_file.read())
        urdf_file.close()
        self.tree = kdl_tree_from_urdf_model(self.robot)

        self.base_link = base_link

        self.joint_chains = []

        # for l_name in BAXTER_LINK_NAMES:
        for l_name in self.arm_links:
            jc = KDLKinematics(self.robot, self.base_link, l_name)
            self.joint_chains.append(jc)

        self.dyn_model = baxterKinematics_info_gain(T=1)
        self.dyn_model.init_transform_pointcloud_kdl()
        self.dyn_model.init_gpflow_gpy()
        # self.traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=False)

    def FK_joint(self, joint_angles, j_index,get_quaternion=True):
        '''
        Method to return task coordinates between base link and any joint
        joint_angles must contain only 0:j_index joints
        '''
        T = self.joint_chains[j_index].forward(joint_angles)
        if(get_quaternion):
            pose=np.zeros(7)
        else:
            pose=np.zeros(6)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])
        if(get_quaternion):
            pose[3:7]=R.GetQuaternion()
        else:
            pose[3:6]=R.GetRPY()
        pose[0:3]=T[0:3,3].ravel()
        return pose

    def vis_collision_robot(self, real_angles):
        # from pcl_mesh_tools.srv import *
        poses = []
        for i in range(11):
            if i <= 6:
                current = real_angles[0:i + 1]
                temp = self.FK_joint(current, i)
                poses.append(temp)
            elif i <= 8:
                current = real_angles[0:8]
                temp = self.FK_joint(current, i)
                poses.append(temp)
            else:
                current = real_angles[0:7]
                current.append(real_angles[8])
                temp = self.FK_joint(current, i)
                poses.append(temp)
        collision_inp = []
        for i in range(len(poses)):
            l_pose = Pose()
            l_pose.position.x = poses[i][0]
            l_pose.position.y = poses[i][1]
            l_pose.position.z = poses[i][2]
            l_pose.orientation.x = poses[i][3]
            l_pose.orientation.y = poses[i][4]
            l_pose.orientation.z = poses[i][5]
            l_pose.orientation.w = poses[i][6]
            collision_inp.append(l_pose)
            # print collision_inp
        try:
            srv_name = 'debug/robot_mesh_viz'
            self.robot_vis = rospy.ServiceProxy(srv_name, collision_checker_msg, persistent=True)
            # Call collision checker:
            resp = self.robot_vis(collision_inp)
            # print resp
        except rospy.ServiceException, e:
            print('service failed')
        print('call success')
        robot_data = []

        return robot_data
    def viz_obj_mesh(self, pose, t='obj_marker', rgb=[0.5, 0.5, 0.5], a=0.4):
        obj_pub = rospy.Publisher(t, Marker, queue_size=1)
        obj_m = Marker()
        obj_m.header.frame_id = 'palm_link'
        obj_m.header.stamp = rospy.Time()
        obj_m.id = 0
        obj_m.type = Marker.MESH_RESOURCE
        obj_m.mesh_resource = self.obj_mesh
        obj_m.action = Marker.ADD
        obj_m.scale.x = 1.0
        obj_m.scale.y = 1.0
        obj_m.scale.z = 1.0
        obj_m.color.a = a
        obj_m.color.r = rgb[0]
        obj_m.color.g = rgb[1]
        obj_m.color.b = rgb[2]
        obj_m.pose = pose.pose
        i = 0
        self.loop_rate = rospy.Rate(10)
        while (not rospy.is_shutdown() and i < 5):
            obj_m.header.stamp = rospy.Time()
            obj_pub.publish(obj_m)
            self.loop_rate.sleep()
            i += 1

    def viz_robot_traj(self, pose_arr, t='robot_marker', rgb=[0.7, 0.5, 0.5], a=0.5):
        obj_pub = rospy.Publisher(t, MarkerArray, queue_size=50)
        m_arr = MarkerArray()
        for j in range(len(pose_arr)):
           poses = pose_arr[j]
           for i in range(len(poses)):
              obj_m = Marker()
              obj_m.header.frame_id = 'base'
              obj_m.header.stamp = rospy.Time()
              obj_m.id = j*12+i
              obj_m.type = Marker.MESH_RESOURCE
              obj_m.mesh_resource = 'package://baxter_description/' + self.arm_meshes[i]
              obj_m.action = Marker.ADD
              obj_m.scale.x = 1.0
              obj_m.scale.y = 1.0
              obj_m.scale.z = 1.0
              obj_m.color.a = a
              obj_m.color.r = rgb[0]
              obj_m.color.g = rgb[1]
              obj_m.color.b = rgb[2]
              obj_m.pose.position.x = poses[i][0]
              obj_m.pose.position.y = poses[i][1]
              obj_m.pose.position.z = poses[i][2]
              obj_m.pose.orientation.x = poses[i][3]
              obj_m.pose.orientation.y = poses[i][4]
              obj_m.pose.orientation.z = poses[i][5]
              obj_m.pose.orientation.w = poses[i][6]
              m_arr.markers.append(obj_m)

        # self.loop_rate = rospy.Rate(10)
        # obj_pub.publish(m_arr)
        # while (not rospy.is_shutdown()):
        #     obj_pub.publish(m_arr)
        #     self.loop_rate.sleep()
        return obj_pub,m_arr

    def viz_traj_mul(self, pose_arr_all, t='robot_marker', rgb=[[0.7, 0.5, 0.5],[0.7, 0.5, 0.5]], a=0.5):
        obj_pub = rospy.Publisher(t, MarkerArray, queue_size=1)
        m_arr = MarkerArray()
        all_traj_num = len(pose_arr_all)
        print('all',all_traj_num)
        for k in range(len(pose_arr_all)):
            pose_arr = pose_arr_all[k]
            for j in range(len(pose_arr)):
                poses = pose_arr[j]
                for i in range(len(poses)):
                    obj_m = Marker()
                    obj_m.header.frame_id = 'base'
                    obj_m.header.stamp = rospy.Time()
                    obj_m.id = k*(all_traj_num) *len(poses)*12 + j * 12 + i
                    obj_m.type = Marker.MESH_RESOURCE
                    obj_m.mesh_resource = 'package://baxter_description/' + self.arm_meshes[i]
                    obj_m.action = Marker.ADD
                    obj_m.scale.x = 1.0
                    obj_m.scale.y = 1.0
                    obj_m.scale.z = 1.0
                    obj_m.color.a = a
                    obj_m.color.r = rgb[k][0]
                    obj_m.color.g = rgb[k][1]
                    obj_m.color.b = rgb[k][2]
                    obj_m.pose.position.x = poses[i][0]
                    obj_m.pose.position.y = poses[i][1]
                    obj_m.pose.position.z = poses[i][2]
                    obj_m.pose.orientation.x = poses[i][3]
                    obj_m.pose.orientation.y = poses[i][4]
                    obj_m.pose.orientation.z = poses[i][5]
                    obj_m.pose.orientation.w = poses[i][6]
                    m_arr.markers.append(obj_m)

        return obj_pub,m_arr




from geometry_msgs.msg import Pose
import tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from pykdl_utils.kdl_kinematics_pointcloud import KDLKinematics_pointcloud
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point


if __name__=='__main__':
    rospy.init_node("vis_client")
    real_angles_2 = [-1.2524953133084402, -0.3601019899561009, -0.04141748127290617, 0.9422476989586154,
                   -0.06557767868210143, 0.9541360500647273, 0.0]
    path='/home/kanrun/catkin_ws/src/baxter_common/baxter_description/'
    urdf_path = path + '/urdf/baxter2_vis.urdf'
    v_hand=visTrainingTraj(path,urdf_path,False,'base')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    limb_interface = baxter_interface.limb.Limb('left')
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]

    temp = [-1.01935909, -0.93444153,  0.7592138 ,  1.45420466, -0.45813642,0.82359786,  0.05361328]
    temp2 = [0, -0.93444153, 0.7592138, 1.45420466, -0.45813642, 0.82359786, 0.05361328]
    u_final = [current_angles,temp,temp2]
    current_angles = np.array(current_angles)
    temp = np.array(temp)




    # u_train = np.load('u_train.npy')
    u_train = np.load('save_best_traj.npy')
    rgb_arr_all = []
    pose_arr_all = []

    '''
    visualize the best training trajectory
    '''

    # for k in range(4):
    #     u_final = u_train[k*5]
    #     pose_arr = []
    #     for j in range(0, len(u_final)):
    #         temp_u = u_final[j]
    #         temp_u = np.append(temp_u, -0.020833)
    #         temp_u = np.append(temp_u, -0.020833)
    #         poses = []
    #         for i in range(12):
    #             if i <= 6:
    #                 current = temp_u[0:i + 1]
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #             elif i <= 7:
    #                 current = temp_u[0:7]
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #             elif i <= 9:
    #                 current = temp_u[0:8]
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #             else:
    #                 current = temp_u[0:7]
    #                 current = np.append(current, temp_u[8])
    #                 temp = v_hand.FK_joint(current, i)
    #                 poses.append(temp)
    #
    #         pose_arr.append(poses)
    #
    #     pose_arr_all.append(pose_arr)
    #     rgb = np.random.random_sample((3,))
    #     rgb_arr_all.append(rgb)

    '''
    visualize the best training trajectory
    '''

    '''
    visualize the best testing trajectory
    '''
    overall = []
    for i in range(len(u_train)):
        u = u_train[i]
        cloud_center = v_hand.dyn_model.FK_pointcloud_center(u)
        cloud_out_ = v_hand.dyn_model.FK_pointcloud_speed(u)
        after_ray, distance = v_hand.dyn_model.call_ray_cast_vis(cloud_out_, cloud_center)
        overall.extend(after_ray)
    from std_msgs.msg import Header
    from sensor_msgs.msg import PointField
    from sensor_msgs.msg import PointCloud2
    HEADER = Header(frame_id='/camera_link')
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)
              ]
    obj_cloud_gpis = pc2.create_cloud(HEADER, fields, overall)
    cloud_pub = rospy.Publisher('gpis_debug', PointCloud2, queue_size=10)

    pose_arr = []
    for j in range(0, len(u_train)):
        temp_u = u_train[j]
        temp_u = np.append(temp_u, -0.020833)
        temp_u = np.append(temp_u, -0.020833)
        poses = []
        for i in range(12):
            if i <= 6:
                current = temp_u[0:i + 1]
                temp = v_hand.FK_joint(current, i)
                poses.append(temp)
            elif i <= 7:
                current = temp_u[0:7]
                temp = v_hand.FK_joint(current, i)
                poses.append(temp)
            elif i <= 9:
                current = temp_u[0:8]
                temp = v_hand.FK_joint(current, i)
                poses.append(temp)
            else:
                current = temp_u[0:7]
                current = np.append(current, temp_u[8])
                temp = v_hand.FK_joint(current, i)
                poses.append(temp)

        pose_arr.append(poses)

    pose_arr_all.append(pose_arr)
    rgb = [0.7, 0.5, 0.5]
    rgb_arr_all.append(rgb)

    '''
    visualize the best testing trajectory
    '''

    obj_pub, m_arr = v_hand.viz_traj_mul(pose_arr_all,t='robot_marker', rgb=rgb_arr_all, a=0.2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cloud_pub.publish(obj_cloud_gpis)
        obj_pub.publish(m_arr)
        rate.sleep()









