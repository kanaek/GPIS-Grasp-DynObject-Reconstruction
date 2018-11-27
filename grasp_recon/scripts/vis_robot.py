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
class VizInHand:
    def __init__(self,mesh_loc, urdf_file_name, init_node=False, base_link="base"):

        self.mesh_loc = mesh_loc
        self.arm_links = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm',
             'left_wrist','left_gripper_base','l_gripper_l_finger','l_gripper_l_finger_tip','l_gripper_r_finger','l_gripper_r_finger_tip']
        self.arm_meshes = ['S0.ply','S1.ply','E0.ply','E1.ply','W0.ply','W1.ply','W2.ply','extended_narrow.ply','half_round_tip.ply','extended_narrow.ply','half_round_tip.ply']
        # initialize node
        if (init_node):
            rospy.init_node('marker_viz')
        # initialize publisher
        self.obj_pub = rospy.Publisher('arm_marker', MarkerArray, queue_size=1)

        urdf_file = file(urdf_file_name, 'r')
        self.robot = Robot.from_xml_string(urdf_file.read())
        urdf_file.close()
        self.tree = kdl_tree_from_urdf_model(self.robot)

        task_space_ik_weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]).tolist()

        # self.base_link = self.robot.get_root()
        self.base_link = base_link

        self.joint_chains = []

        # for l_name in BAXTER_LINK_NAMES:
        for l_name in self.arm_links:
            jc = KDLKinematics(self.robot, self.base_link, l_name)
            self.joint_chains.append(jc)
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
        obj_pub = rospy.Publisher(t, MarkerArray, queue_size=1)
        m_arr = MarkerArray()
        # poses = pose_arr.poses
        for j in range(len(pose_arr)):
           poses = pose_arr[j]
           for i in range(len(poses)):
              obj_m = Marker()
              obj_m.header.frame_id = 'base'
              obj_m.header.stamp = rospy.Time()
              obj_m.id = j*7+i
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

    def get_signed_distance(self,real_angles):
        #from pcl_mesh_tools.srv import *
        srv_name = 'collision_checker'
        srv_name2 = 'collision_checker/update_env_cloud'
        self.collision_checker = rospy.ServiceProxy(srv_name, collision_checker_msg, persistent=True)
        self.update_env_cloud = rospy.ServiceProxy(srv_name2, update_env, persistent=True)
        temp = self.update_env_cloud()
        poses = []
        resp = None
        for i in range(11):
            if i>=7 and i <= 8:
                current = real_angles[0:8]
                temp = self.FK_joint(current, i)
                poses.append(temp)
            elif i>=9 and i<=10:
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
            resp = self.collision_checker(collision_inp)
            # print resp
        except rospy.ServiceException, e:
            print('service failed')
        print('call success')
        robot_data = []
        for i in range(len(resp.link_data)):
            print('iiiiiiiiiiiiiiiii',i)
            link_data = []
            for j in range(len(resp.link_data[i].c_data)):
                # col_data = [resp.link_data[i].c_data[j].collision, np.array(
                #     [float(resp.link_data[i].c_data[j].point[0]), float(resp.link_data[i].c_data[j].point[1]),
                #      float(resp.link_data[i].c_data[j].point[2])]),
                #             np.array(resp.link_data[i].c_data[j].dir), resp.link_data[i].c_data[j].distance]
                col_data = [resp.link_data[i].c_data[j].collision,
                            np.array(resp.link_data[i].c_data[j].dir), resp.link_data[i].c_data[j].distance]
                link_data.append(col_data)
            print(link_data)
            robot_data.append(link_data)

        return robot_data

def to_kdl_frame_quaternion(pose):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(pose[3], pose[4],
                                                 pose[5], pose[6]),
                        PyKDL.Vector(pose[0],pose[1],
                                     pose[2]))
def to_kdl_frame_rpy(pose):
    return PyKDL.Frame(PyKDL.Rotation.RPY(pose[3], pose[4],
                                                 pose[5]),
                        PyKDL.Vector(pose[0],pose[1],
                                     pose[2]))


def interpolate_joints(T, start,finish):
    linear_poses = []
    linear_poses.append(start)
    for i in range(T):
        sample_pose = start + (finish - start) * float(i + 1) / T
        linear_poses.append(sample_pose)
    linear_poses.append(finish)
    return linear_poses


from geometry_msgs.msg import Pose
import tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from pykdl_utils.kdl_kinematics_pointcloud import KDLKinematics_pointcloud
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, t.transform.translation.y,
                                    t.transform.translation.z))


def transform_cloud(cloud, transform):
    t_kdl = transform_to_kdl(transform)
    points_out = []
    for p_in in cloud:
        p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
        points_out.append((p_out[0], p_out[1], p_out[2]))
    return points_out


if __name__=='__main__':

    real_angles_2 = [-1.2524953133084402, -0.3601019899561009, -0.04141748127290617, 0.9422476989586154,
                   -0.06557767868210143, 0.9541360500647273, 0.0]
    #35195558377974, 'left_w2': 2.03444201993303, 'left_e0': 0.6645971763513555, 'left_e1': 1.3334127998693959, 'left_s0': -1.1650584083989717, 'left_s1': -0.7907670961549308}
    path='/home/kanrun/catkin_ws/src/baxter_common/baxter_description/'
    urdf_path = path + '/urdf/baxter2_vis.urdf'
    v_hand=VizInHand(path,urdf_path,False,'base')
    rospy.init_node('marker_viz')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    limb_interface = baxter_interface.limb.Limb('left')
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    print(current_angles)

    rospy.wait_for_service('/collision')
    collision_checking=rospy.ServiceProxy('/collision',collsion_check)
    print('finish waiting, service online')
    current = np.append(current_angles, -0.020833)
    temp = v_hand.FK_joint(current, 10)
    print('temp_pose',temp)
    l_pose = Pose()
    l_pose.position.x = temp[0]
    l_pose.position.y = temp[1]
    l_pose.position.z = temp[2]
    l_pose.orientation.x = temp[3]
    l_pose.orientation.y = temp[4]
    l_pose.orientation.z = temp[5]
    l_pose.orientation.w = temp[6]


    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform_optical_left_gripper = tf_buffer.lookup_transform('left_gripper',
                                           'camera_rgb_optical_frame',  # source frame
                                           rospy.Time(0),  # get the tf at first available time
                                           rospy.Duration(1.0))  # wait for 1 second

    base_camera = tf_buffer.lookup_transform('camera_link',
                                             'base',  # source frame
                                             rospy.Time(0),  # get the tf at first available time
                                             rospy.Duration(1.0))

    t_kdl = transform_to_kdl(base_camera)
    camera_origin = t_kdl * PyKDL.Vector(0., 0., 0.)
    print('camera_origin',camera_origin) #('camera_origin', [      1.2877,   -0.731615,    0.296133])
    print('camera_origin', base_camera)



    # test_cloud = np.load('new_xx.npy')
    #
    # cloud = transform_cloud(test_cloud, transform_optical_left_gripper)
    #
    # rospy.wait_for_service('/cloud_bounding')
    # cloud_bound = rospy.ServiceProxy('/cloud_bounding', cloud_bounding)
    # print('finish waiting, cloud bounding service online')
    # HEADER = Header(frame_id='/left_gripper')
    # fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #           PointField('y', 4, PointField.FLOAT32, 1),
    #           PointField('z', 8, PointField.FLOAT32, 1)
    #           ]
    # obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud)
    # res = cloud_bound(obj_cloud_gpis)
    # print('res',res)
    # bounding_pub = rospy.Publisher('bounding_box', Marker, queue_size=1)
    # obj_m = Marker()
    # obj_m.header.frame_id = 'left_gripper'
    # # obj_m.header.stamp = ''
    # obj_m.id = 3
    # obj_m.type = 1
    # obj_m.action = Marker.ADD
    # obj_m.scale.x = res.width
    # obj_m.scale.y = res.height
    # obj_m.scale.z = res.depth
    # obj_m.color.a = 0.5
    # obj_m.color.r = 1
    # obj_m.color.g = 0
    # obj_m.color.b = 0
    # obj_m.pose.position.x = res.p_center.position.x
    # obj_m.pose.position.y = res.p_center.position.y
    # obj_m.pose.position.z = res.p_center.position.z
    # obj_m.pose.orientation.x = res.p_center.orientation.x
    # obj_m.pose.orientation.y = res.p_center.orientation.y
    # obj_m.pose.orientation.z = res.p_center.orientation.z
    # obj_m.pose.orientation.w = res.p_center.orientation.w
    #
    # '''
    # add collision center
    # '''
    # quaternion = (
    #     res.p_center.orientation.x,
    #     res.p_center.orientation.y,
    #     res.p_center.orientation.z,
    #     res.p_center.orientation.w)
    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # chain = KDLKinematics_pointcloud(v_hand.robot, 'base', 'left_gripper', 'left_gripper')
    # pose = [res.p_center.position.x,res.p_center.position.y,res.p_center.position.z,euler[0],euler[1],euler[2]]
    # chain.add_new_points_to_tree(pose, 'collision_center')
    #
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    #     T=chain.forward_pointcloud(current_angles, end_link='collision_center', base_link='base')
    #     pose=np.zeros(7)
    #     R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])
    #     pose[3:7]=R.GetQuaternion()
    #     pose[0:3]=T[0:3,3].ravel()
    #     obj_m.pose.position.x = pose[0]
    #     obj_m.pose.position.y = pose[1]
    #     obj_m.pose.position.z = pose[2]
    #     obj_m.pose.orientation.x = pose[3]
    #     obj_m.pose.orientation.y = pose[4]
    #     obj_m.pose.orientation.z = pose[5]
    #     obj_m.pose.orientation.w = pose[6]
    #     obj_m.header.frame_id = 'base'
    #
    #     l_pose.position.x = pose[0]
    #     l_pose.position.y = pose[1]
    #     l_pose.position.z = pose[2]
    #     l_pose.orientation.x = pose[3]
    #     l_pose.orientation.y = pose[4]
    #     l_pose.orientation.z = pose[5]
    #     l_pose.orientation.w = pose[6]
    #
    #
    #     pose_arr = PoseArray()
    #     current_angles = np.append(current_angles, 0.020833)
    #     current_angles = np.append(current_angles, -0.020833)
    #
    #     for i in range(12):
    #         if i <= 6:
    #             current = current_angles[0:i + 1]
    #             temp = v_hand.FK_joint(current, i)
    #             # poses.append(temp)
    #         elif i <= 7:
    #             current = current_angles[0:7]
    #             temp = v_hand.FK_joint(current, i)
    #         elif i <= 8:
    #             current = current_angles[0:8]
    #             temp = v_hand.FK_joint(current, i)
    #             # poses.append(temp)
    #         else:
    #             current = current_angles[0:7]
    #             current = np.append(current, current_angles[8])
    #             temp = v_hand.FK_joint(current, i)
    #
    #         pose = Pose()
    #         pose.position.x = temp[0]
    #         pose.position.y = temp[1]
    #         pose.position.z = temp[2]
    #         pose.orientation.x = temp[3]
    #         pose.orientation.y = temp[4]
    #         pose.orientation.z = temp[5]
    #         pose.orientation.w = temp[6]
    #
    #         pose_arr.poses.append(pose)
    #
    #     current = current_angles[0:7]
    #     temp = v_hand.FK_joint(current, 6)
    #     cloud_out_ = []
    #     pose = np.zeros(3)
    #     pose[0:3] = temp[0:3]
    #     cloud_out_.append(pose)
    #
    #     cloud_out_ = transform_cloud(cloud_out_, base_camera)
    #     point_center = Point()
    #     point_center.x = cloud_out_[0][0]
    #     point_center.y = cloud_out_[0][1]
    #     point_center.z = cloud_out_[0][2]
    #     res = collision_checking(l_pose,pose_arr,point_center)
    #
    #
    #
    #     print('res',res)
    #     bounding_pub.publish(obj_m)
    #     rate.sleep()




    # print('in collsion:', res.success)

    # current_angles[6] = current_angles[6]
    # print(current_angles)
    # temp = [-1.01935909, -0.93444153,  0.7592138 ,  1.45420466, -0.45813642,0.82359786,  0.05361328]
    # u_final = [current_angles,temp]
    # current_angles = np.array(current_angles)
    # temp = np.array(temp)
    #
    #
    #
    #
    # linear_poses = interpolate_joints(5,current_angles,temp)
    # traj_server = trajectoryServer(20)
    # u_final_list = list(linear_poses)
    #
    # for j in range(0, len(u_final_list)):
    #     temp = u_final_list[j]
    #     temp = np.append(temp, 0.020833)
    #     temp = np.append(temp, -0.020833)
    #     u_final_list[j] = temp
    # pub_tr, disp_tr = traj_server.send_traj(u_final_list)
    #
    #
    #
    # pose_arr = []
    # for j in range(0, len(u_final)):
    #         temp_u = u_final[j]
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
    # obj_pub, m_arr = v_hand.viz_robot_traj(pose_arr)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     pub_tr.publish(disp_tr)
    #     obj_pub.publish(m_arr)
    #     rate.sleep()









