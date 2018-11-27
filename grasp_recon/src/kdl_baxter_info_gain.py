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
from geometry_msgs.msg import Point
import GPy as gp
import GPflow

class baxterKinematics_info_gain:
    def __init__(self,delta_t=1.0/60.0,T=1):
        ''' delta_t= timestep
            T= total time steps
            fingers=[0-4],0=index,1=middle,2=ring,3=thumb        
            The model assumes all fingers have same number of joints.
        '''
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
        robot = Robot.from_xml_string(urdf_file.read())
        self.chain = KDLKinematics_pointcloud(robot, 'base', 'left_gripper', 'left_gripper')

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



    def init_transform_pointcloud_kdl(self):
        self.test_cloud = np.load('new_xx_all.npy')
        cloud_center = []
        cloud_out = self.transform_cloud(self.test_cloud, self.transform_optical_frame_gripper)
        for i in range(len(cloud_out)):
            if i == 0:
                child_name_cloud = str(i)
                pose = cloud_out[i]
                cloud_center = np.array(list(pose))
                # print(i)
                self.chain.add_new_points_to_tree(pose, child_name_cloud)
            else:
                child_name_cloud = str(i)
                pose = cloud_out[i]
                cloud_center = cloud_center + np.array(list(pose))
                self.chain.add_new_points_to_tree(pose, child_name_cloud)

        self.num_points = len(cloud_out)
        cloud_center = np.dot(cloud_center, 1.0 / len(cloud_out))
        cloud_center = tuple(cloud_center)
        self.chain.add_new_points_to_tree(cloud_center, 'cloud_center')


    def init_gpflow_gpy(self,lengthscales = 0.07):
        '''
        init gpflow
        :return: 
        '''
        k = GPflow.kernels.RBF(input_dim=3, lengthscales=lengthscales)
        # msstate = np.load('saveModelState.npy')
        X = np.load('X.npy')
        Y = np.load('Y.npy')
        width = np.load('width.npy')
        height = np.load('height.npy')
        depth = np.load('depth.npy')
        meanf = GPflow.mean_functions.Ellipsoidal(width, height, depth)
        self.gpflow = GPflow.gpr.GPR(X, Y, kern=k, mean_function=meanf)
        self.gpflow.likelihood.variance = 0.1

        '''
        init gpy
        '''
        k_2 = gp.kern.RBF(input_dim=3, variance=2.0, lengthscale=lengthscales)
        self.gpy = gp.models.GPRegression(X, Y, kernel=k_2)
        self.gpy.likelihood.variance = 0.1

    def FK_pointcloud_center(self,u):
        cloud_out_ = []
        test_link = self.chain.forward_pointcloud(u, end_link='cloud_center', base_link='base')
        pose = np.zeros(3)
        pose[0:3] = test_link[0:3, 3].ravel()
        cloud_out_.append(pose)

        cloud_out_ = self.transform_cloud(cloud_out_, self.base_camera)
        return cloud_out_


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
        for i in range(self.num_points):
            virtual_link = str(i)
            test_link = self.chain.forward_pointcloud(u, end_link=virtual_link, base_link='base')
            pose = np.zeros(4)
            pose[0:3] = test_link[0:3, 3].ravel()
            pose[3] = self.test_cloud[i][3]
            cloud_out_.append(pose)

        return cloud_out_

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

    
