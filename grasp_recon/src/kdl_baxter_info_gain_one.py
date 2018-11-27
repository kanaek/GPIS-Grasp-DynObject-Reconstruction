# This file creates an instance of the KUKA LBR4 robot with FK and Jacobian using KDL.
import rospy
from rospkg import RosPack
#import manipulator_model_2
import manipulator_model_left
import numpy as np
from numpy import cos,sin
from numpy.linalg import inv
from numpy.linalg import pinv
import PyKDL
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics_pointcloud import KDLKinematics_pointcloud
import tf2_ros
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from optimization_baxter.srv import raycast
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point
import GPy as gp
# import GPflow
import baxter_interface
import tf

import sys
rp = RosPack()
rp.list()
path_collision_pkg=rp.get_path('collision_check')
sys.path.insert(0,path_collision_pkg)

from collision_check.srv import collsion_check
from collision_check.srv import cloud_bounding

from geometry_msgs.msg import Pose,PoseArray
from pykdl_utils.kdl_kinematics import KDLKinematics

class baxterKinematics_info_gain:
    def __init__(self,delta_t=1.0/60.0,T=1):

        self.n=6
        self.m=7
        self.n_links=8
        self.T=T
        self.delta_t=delta_t
        self.finite_difference = 0.1
        self.finite_array = np.ones(self.m)
        self.finite_array[:] = self.finite_difference
        rp = RosPack()
        packages = rp.list()
        path = rp.get_path('baxter_description')
        path= path + '/urdf/baxter2_correct_2.urdf'

        urdf_file = file(path, 'r')
        temp = urdf_file.read()
        robot = Robot.from_xml_string(temp)
        self.chain = KDLKinematics_pointcloud(robot, 'base', 'left_gripper', 'left_gripper')
        # self.vis_chain = KDLKinematics_pointcloud(robot, 'base', 'left_gripper', 'left_gripper')

        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        self.transform_optical_frame_gripper = tf_buffer.lookup_transform('left_gripper',
                                               'camera_rgb_optical_frame',  # source frame
                                               rospy.Time(0),  # get the tf at first available time
                                               rospy.Duration(1.0))  # wait for 1 second
        self.base_camera = tf_buffer.lookup_transform('camera_link',
                                               'base',  # source frame
                                               rospy.Time(0),  # get the tf at first available time
                                               rospy.Duration(1.0))

        def transform_base(t):
            return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                         t.transform.rotation.z, t.transform.rotation.w),
                               PyKDL.Vector(t.transform.translation.x, t.transform.translation.y,
                                            t.transform.translation.z))


        # t_kdl = transform_base(self.base_camera)
        # self.camera_origin = t_kdl * PyKDL.Vector(0., 0., 0.)
        self.camera_origin = [1.2877,-0.731615,0.296133]



        self.transform_optical_base = tf_buffer.lookup_transform('base',
                                               'camera_rgb_optical_frame',  # source frame
                                               rospy.Time(0),  # get the tf at first available time
                                               rospy.Duration(1.0))



        self.kdlModel=manipulator_model_left.ManipulatorSimpleModel(path,base_link='base')
        low_bounds=[]
        up_bounds=[]
        
        low_bounds+=list(self.kdlModel.chain.get_joint_limits()[0])
        up_bounds+=list(self.kdlModel.chain.get_joint_limits()[1])
        self.bounds=np.array([low_bounds,up_bounds])
        self.link_offset=[[0.0,0.0,0.0],[0.0,0.0,0.0],
                          [0.0,0.0,0.0],[0.0,0.0,0.],
                          [0.0,0.0,0.0],[0.0,-0.0,0.],
                          [0.0,0.0,0.0],[0.0,0.0,0.0]]
        
        self.DOF=7

        self.joint_chains = []
        self.arm_links = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm',
             'left_wrist','left_gripper_base','l_gripper_l_finger','l_gripper_l_finger_tip','l_gripper_r_finger','l_gripper_r_finger_tip']

        # for l_name in BAXTER_LINK_NAMES:
        for l_name in self.arm_links:
            jc = KDLKinematics(robot, 'base', l_name)
            self.joint_chains.append(jc)


        '''
        init collision checking service
        '''
        rospy.wait_for_service('/collision')
        self.collision_checking = rospy.ServiceProxy('/collision', collsion_check)




    def init_transform_pointcloud_kdl(self):
        self.test_cloud = np.load('new_xx_all.npy')
        self.real_test_cloud = np.load('new_xx.npy')
        cloud_center = []
        cloud_out = self.transform_cloud(self.test_cloud, self.transform_optical_frame_gripper)
        self.cloud = self.transform_cloud(self.test_cloud, self.transform_optical_base)
        for i in range(len(cloud_out)):
            if i == 0:
                child_name_cloud = str(i)
                pose = cloud_out[i]
                # cloud_center = np.array(list(pose))
                self.chain.add_new_points_to_tree(pose, child_name_cloud)
            else:
                child_name_cloud = str(i)
                pose = cloud_out[i]
                # cloud_center = cloud_center + np.array(list(pose))
                self.chain.add_new_points_to_tree(pose, child_name_cloud)

        cloud_out_real = self.transform_cloud(self.real_test_cloud, self.transform_optical_frame_gripper)
        for i in range(len(cloud_out_real)):
            if i == 0:
                # child_name_cloud = str(i)
                pose = cloud_out_real[i]
                cloud_center = np.array(list(pose))
                # self.vis_chain.add_new_points_to_tree(pose, child_name_cloud)
            else:
                # child_name_cloud = str(i)
                pose = cloud_out_real[i]
                cloud_center = cloud_center + np.array(list(pose))
                # self.vis_chain.add_new_points_to_tree(pose, child_name_cloud)

        self.num_points = len(cloud_out)
        # self.vis_num_points = len(cloud_out_real)
        cloud_center = np.dot(cloud_center, 1.0 / len(cloud_out_real))
        cloud_center = tuple(cloud_center)
        self.chain.add_new_points_to_tree(cloud_center, 'cloud_center')


        '''
        init collisioin center and init bounding box through PCA in FCL
        '''
        rospy.wait_for_service('/cloud_bounding')
        cloud_bound = rospy.ServiceProxy('/cloud_bounding', cloud_bounding)
        print('finish waiting, cloud bounding service online')
        HEADER = Header(frame_id='/left_gripper')
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)
                  ]
        obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out)
        res = cloud_bound(obj_cloud_gpis)
        print('res', res)
        quaternion = (
            res.p_center.orientation.x,
            res.p_center.orientation.y,
            res.p_center.orientation.z,
            res.p_center.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        pose = [res.p_center.position.x, res.p_center.position.y, res.p_center.position.z, euler[0], euler[1], euler[2]]
        self.chain.add_new_points_to_tree(pose, 'collision_center')








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


    '''
    collision checking between robot arm , GPIS model and table
    '''
    def collision_table(self,u_input,p_center):
        # T = self.chain.forward_pointcloud(u_input, end_link='collision_center', base_link='base')
        # pose = np.zeros(7)
        # R = PyKDL.Rotation(T[0, 0], T[0, 1], T[0, 2], T[1, 0], T[1, 1], T[1, 2], T[2, 0], T[2, 1], T[2, 2])
        # pose[3:7] = R.GetQuaternion()
        # pose[0:3] = T[0:3, 3].ravel()
        l_pose = Pose()
        l_pose.position.x = 0
        l_pose.position.y = 0
        l_pose.position.z = 0
        l_pose.orientation.x = 0
        l_pose.orientation.y = 0
        l_pose.orientation.z = 0
        l_pose.orientation.w = 1

        pose_arr = PoseArray()
        u_input = np.append(u_input, 0.020833)
        u_input = np.append(u_input, -0.020833)

        for i in range(12):
            if i <= 6:
                current = u_input[0:i + 1]
                temp = self.FK_joint(current, i)
            elif i <= 7:
                current = u_input[0:7]
                temp = self.FK_joint(current, i)
            elif i <= 8:
                current = u_input[0:8]
                temp = self.FK_joint(current, i)
            else:
                current = u_input[0:7]
                current = np.append(current, u_input[8])
                temp = self.FK_joint(current, i)

            pose = Pose()
            pose.position.x = temp[0]
            pose.position.y = temp[1]
            pose.position.z = temp[2]
            pose.orientation.x = temp[3]
            pose.orientation.y = temp[4]
            pose.orientation.z = temp[5]
            pose.orientation.z = temp[6]

            pose_arr.poses.append(pose)
        res = self.collision_checking(l_pose, pose_arr, p_center)

        return res

    def init_gpflow_gpy(self,lengthscales = 0.1):
        '''
        init gpflow
        :return: 
        '''
        # k = GPflow.kernels.RBF(input_dim=3, lengthscales=lengthscales)
        # msstate = np.load('saveModelState.npy')

        X = np.load('X.npy')
        Y = np.load('Y.npy')
        # width = np.load('width.npy')
        # height = np.load('height.npy')
        # depth = np.load('depth.npy')
        # # meanf = GPflow.mean_functions.Ellipsoidal(width, height, depth)
        # k = GPflow.kernels.RBF(input_dim=3, variance=1.0, lengthscales=0.1)
        # meanf = GPflow.mean_functions.Constant(1)
        # self.gpflow = GPflow.gpr.GPR(X, Y, kern=k, mean_function=meanf)
        # self.gpflow.likelihood.variance = 0.1

        '''
        init gpy
        '''
        k_2 = gp.kern.RBF(input_dim=3, variance=2.0, lengthscale=lengthscales)
        self.gpy = gp.models.GPRegression(X, Y, kernel=k_2)
        self.gpy.likelihood.variance = 0.1

        limb_interface = baxter_interface.limb.Limb('left')
        self.current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]

        test_link = self.chain.forward_pointcloud(self.current_angles, end_link='cloud_center', base_link='base')
        self.pose = np.zeros(3)
        self.pose[0:3] = test_link[0:3, 3].ravel()
        self.inv_link = np.linalg.inv(test_link)



    def FK_pointcloud_center(self,u):
        cloud_out_ = []
        test_link = self.chain.forward_pointcloud(u, end_link='cloud_center', base_link='base')
        pose = np.zeros(3)
        pose[0:3] = test_link[0:3, 3].ravel()
        cloud_out_.append(pose)

        cloud_out_ = self.transform_cloud(cloud_out_, self.base_camera)
        return cloud_out_

    '''
    new pointcloud transformation method
    '''
    def FK_pointcloud_speed(self, u):
        test_link = self.chain.forward_pointcloud(u, end_link='cloud_center', base_link='base')
        trans_matrix = np.dot(test_link,self.inv_link)
        # pose = np.zeros(3)
        # pose[0:3] = test_link[0:3, 3].ravel()
        # diff = pose - self.pose
        # temp_cloud = np.copy(self.cloud)
        # temp_cloud = temp_cloud + diff

        temp_cloud = np.copy(self.cloud)
        b = np.ones(temp_cloud.shape[0])

        temp_cloud = temp_cloud.transpose()
        temp_stack = np.row_stack((temp_cloud,b))
        temp_result = np.dot(trans_matrix,temp_stack)
        temp_result = temp_result.transpose()
        temp_result = temp_result[:,0:3]
        temp_result = np.asarray(temp_result)
        cloud_out_ = self.transform_cloud(temp_result, self.base_camera)

        return cloud_out_




    def FK_pointcloud_center_pose(self,u,get_quaternion=False):
        T=self.chain.forward_pointcloud(u, end_link='cloud_center', base_link='base')
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


    def FK_pointcloud(self,u):
        cloud_out_ = []
        for i in range(self.num_points):
            virtual_link = str(i)
            test_link = self.chain.forward_pointcloud(u, end_link=virtual_link, base_link='base')
            pose = np.zeros(3)
            pose[0:3] = test_link[0:3, 3].ravel()
            cloud_out_.append(pose)

        cloud_out_ = self.transform_cloud(cloud_out_, self.base_camera)
        return cloud_out_

    def vis_cloud(self,u):
        cloud_out_ = []
        for i in range(self.vis_num_points):
            virtual_link = str(i)
            test_link = self.vis_chain.forward_pointcloud(u, end_link=virtual_link, base_link='base')
            pose = np.zeros(3)
            pose[0:3] = test_link[0:3, 3].ravel()
            cloud_out_.append(pose)
        after_ray1 = []
        for i in range(self.vis_num_points):
            temp = list(cloud_out_[i][0:3])
            temp.append(self.real_test_cloud[i][3])
            after_ray1.append(temp)

        after_ray1 = np.array(after_ray1)
        HEADER = Header(frame_id='/base')
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 16, PointField.FLOAT32, 1)
                  ]
        after_gpis = pc2.create_cloud(HEADER, fields, after_ray1)
        cloud_pub = rospy.Publisher('gpis_debug_vis', PointCloud2, queue_size=10)
        # cloud_pub.publish(after_gpis)
        return cloud_pub,after_gpis

    def call_ray_cast(self,cloud_out_,cloud_center,return_index = False):
        point_center = Point()
        point_center.x = cloud_center[0][0]
        point_center.y = cloud_center[0][1]
        point_center.z = cloud_center[0][2]
        HEADER = Header(frame_id='/camera_link')
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)
                  ]
        obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_)
        rospy.wait_for_service('raycast')
        raycast_srv = rospy.ServiceProxy('raycast', raycast)
        raycast_resp = raycast_srv(obj_cloud_gpis,point_center)
        temp_list = raycast_resp.array
        temp_list = list(set(temp_list))
        distance = raycast_resp.distance
        if return_index == False:
            after_ray = []
            for i in range(len(temp_list)):
               after_ray.append(list(self.test_cloud[temp_list[i]][0:3]))

            after_ray = np.array(after_ray)
            return after_ray,distance
        else:
            after_ray = []
            for i in range(len(temp_list)):
                after_ray.append(list(self.test_cloud[temp_list[i]][0:3]))

            after_ray = np.array(after_ray)
            return after_ray,temp_list,distance

    def call_ray_cast_return_num(self, cloud_out_, cloud_center):
            point_center = Point()
            point_center.x = cloud_center[0][0]
            point_center.y = cloud_center[0][1]
            point_center.z = cloud_center[0][2]
            HEADER = Header(frame_id='/camera_link')
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)
                      ]
            obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_)
            # rospy.wait_for_service('raycast')
            raycast_srv = rospy.ServiceProxy('raycast', raycast)
            raycast_resp = raycast_srv(obj_cloud_gpis, point_center)
            temp_list = raycast_resp.array
            temp_list = list(set(temp_list))
            distance = raycast_resp.distance
            return temp_list, distance

    def call_ray_cast_vis(self, cloud_out_, cloud_center):
            point_center = Point()
            point_center.x = cloud_center[0][0]
            point_center.y = cloud_center[0][1]
            point_center.z = cloud_center[0][2]
            HEADER = Header(frame_id='/camera_link')
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)
                      ]
            obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_)
            # rospy.wait_for_service('raycast')
            raycast_srv = rospy.ServiceProxy('raycast', raycast)
            raycast_resp = raycast_srv(obj_cloud_gpis, point_center)
            temp_list = raycast_resp.array
            temp_list = list(set(temp_list))
            distance = raycast_resp.distance
            after_ray = []
            for i in range(len(temp_list)):
                after_ray.append(list(cloud_out_[temp_list[i]][0:3]))

            return after_ray, distance

    def append_pointcloud(self,temp_list):
        after_ray = []
        for i in range(len(temp_list)):
            after_ray.append(list(self.test_cloud[temp_list[i]][0:3]))

        after_ray = np.array(after_ray)
        return after_ray


    #unfinished part
    def jacobian_pointcloud(self,u):
        for i in range(self.num_points):
            virtual_link = str(i)
            test_jacobian = self.chain.jacobian_pointcloud(u, pos=None,base_link='base',end_link=virtual_link)
            # currently don't know what to do


    def jacobian_pointcloud_center(self,u):
        test_jacobian = self.chain.jacobian_pointcloud(u, pos=None,base_link='base',end_link='cloud_center')
        return test_jacobian


    #transform the pointcloud
    def transform_to_kdl(self,t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                     t.transform.rotation.z, t.transform.rotation.w),
                           PyKDL.Vector(t.transform.translation.x, t.transform.translation.y,
                                        t.transform.translation.z))

    def transform_cloud(self,cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        for p_in in cloud:
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append((p_out[0], p_out[1], p_out[2]))
        return points_out


    '''
    conventional code
    '''
    def end_effector_pose_array(self,u,get_quaternion=False):
        T=self.kdlModel.FK(u)
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



    def link_pose_array(self,u,link_num):
        T=self.kdlModel.FK_joint(u[0:link_num],link_num)
        pose=np.zeros(7)
        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],T[1,0],T[1,1],T[1,2],T[2,0],T[2,1],T[2,2])

        # Normalize quaternion:
        quat=R.GetQuaternion()
        #print np.linalg.norm(quat)
        #quat=quat/np.linalg.norm(quat)

        pose[3:7]=quat
        pose[0:3]=T[0:3,3].ravel()+self.link_offset[link_num]
        return pose

    def all_link_poses(self,u):
        r_link_poses=[]
        offset=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        for i in range(self.n_links):
            r_pose=self.link_pose_array(u,i)
            r_link_poses.append(r_pose+offset)
        return r_link_poses
    
    def predict_T(self,x0,u,w):
        # Predicts the next state
        T=self.T
        x_n=np.array([[0.0 for z in range(self.n)] for i in range(T)])
        for i in range(T):
            x_n[i,:]=self.predict(np.ravel(u[i,:]))
        return x_n

    def predict(self,u):
        x_k_plus_1=self.end_effector_pose_array(u)
        
        return x_k_plus_1

    def end_effector_pose(self,u,EE_index=0):

        T=self.kdlModel.FK(u,EE_index)
        return T


    def jacobian(self,u):
        J=self.kdlModel.Jacobian(u)[0:3,:]
        return J
    def jacobian_orient(self,u):
        J=self.kdlModel.Jacobian(u)[3:6,:]
        return J
    def jacobian_full(self,u):
        J=self.kdlModel.Jacobian(u)
        return J
    
    def link_jacobian(self,u,link_index):
        J=self.kdlModel.Jacobian_joint(u[0:link_index],link_index)
        return J

    def predict_xdot(self,x,u):
        # given the current object pose and the new input, find the velcity
        #des_finger_pose=np.ravel(self.linear_finger_poses[t_step])
        x_pose=self.end_effector_pose_array(u[3*4:3*4+4])
        x_dot=(x-x_pose)/self.delta_t
        x_new=x_dot
        return x_new

    
