#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
#import moveit_commander
import rospy
import tf2_ros
import tf
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
from geometry_msgs.msg import Pose
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
baxter_urdf_path=rp.get_path('baxter_description')
baxter_urdf_path = baxter_urdf_path+'/urdf/baxter2_correct_2.urdf'
from optimization_baxter.srv import raycast
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
print baxter_urdf_path
import pyOpt


# def objfunc(x):
#     f = -x[0] * x[1] * x[2]
#     g = [0.0] * 2
#     g[0] = x[0] + 2. * x[1] + 2. * x[2] - 72.0
#     g[1] = -x[0] - 2. * x[1] - 2. * x[2]
#
#     fail = 0
#     return f, g, fail
#
# def grad_obj_func(q,f,g):
#         g_obj = [0,0,0]
#         g_con = [0,0]
#         fail=0
#
#         return g_obj,[g_con],fail
# # =============================================================================
# #
# # =============================================================================
#
# # Instantiate Optimization Problem
# opt_prob=pyOpt.Optimization('TP37 Constrained Problem',objfunc)
# method=getattr(pyOpt,'SLSQP')
# optimizer=method(options=self.opt_options)
# optimizer=method(options='SLSQP')
# [fstr,xstr,inform]=optimizer(opt_prob,sens_type=self.grad_obj_func)
# opt_prob.addObj('f')
# opt_prob.addConGroup('g',2, type='i',upper=np.array([2,2]))
# opt_prob.addVar('x1', 'c', lower=0.0, upper=42.0, value=10.0)
# opt_prob.addVar('x2', 'c', lower=0.0, upper=42.0, value=10.0)
# opt_prob.addVar('x3', 'c', lower=0.0, upper=42.0, value=10.0)
# # opt_prob.addCon('g1', 'i')
# # opt_prob.addCon('g2', 'i')
# print opt_prob

# Instantiate Optimizer (PSQP) & Solve Problem
# psqp = pyOpt.SLSQP()
# psqp.setOption('IPRINT', 0)
# psqp(opt_prob, sens_type='FD')
# print opt_prob.solution(0)


# rospy.init_node("grasp_client")
# HEADER = Header(frame_id='/camera_link')
# fields = [PointField('x', 0, PointField.FLOAT32, 1),
#           PointField('y', 4, PointField.FLOAT32, 1),
#           PointField('z', 8, PointField.FLOAT32, 1)
#           ]
# cloud_out = [[0.1,0.2,0.3],[0.1,0.2,0.4]]
# obj_cloud_gpis = pc2.create_cloud(HEADER,fields, cloud_out)
# rospy.wait_for_service('raycast')
# raycast_srv = rospy.ServiceProxy('raycast', raycast)
# raycast_resp = raycast_srv(obj_cloud_gpis)
# print(raycast_resp.success)
# print('finish')
# ray_cast_cloud = pc2.read_points(raycast_resp.ray_cast_cloud, skip_nans=False)
# ray_cast_cloud = np.array(list(ray_cast_cloud))

# import GPy as gp
# import numpy as np
# k = gp.kern.RBF(input_dim=3, variance=1.0, lengthscale=0.07)
# X = np.load('X.npy')
# X_star = np.load('new_xx.npy')
# C = k.K(X,X)
# C_1 = k.K(X,X_star)
# C_2 = k.K(X_star,X)
# C_3 = k.K(X_star,X_star)
# temp_var = np.dot(2.0 * np.pi * np.exp(1), C_3)
# info_gain = np.linalg.det(temp_var)*0.5
# print(C_1)
# test = [1,2,3]
# test = np.dot(-1.0,test)
# print(test)


'''
///////////////////////////////////////////////////////
generate GPIS model
'''
rospy.init_node("tf_client")
data = np.array([[1,2],[2,3],[3,4]])
avg = np.mean(data,axis=0)
print('avg',avg)
import PyKDL as kdl
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_kinematics import KDLKinematics
from tabletop_obj_segmentation.srv import SegmentGraspObject
urdf_file = file(baxter_urdf_path, 'r')
robot = Robot.from_xml_string(urdf_file.read())
chain = KDLKinematics(robot, 'base', 'left_gripper_mid')
root = robot.get_root()
rospy.wait_for_service('object_segmenter')
obj_pose_srv = rospy.ServiceProxy('object_segmenter', SegmentGraspObject)
obj_resp = obj_pose_srv(False)
pose = obj_resp.obj.pose

'''
transform the pose
'''
import tf2_geometry_msgs
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
tf_listener = tf2_ros.TransformListener(tf_buffer)
transform = tf_buffer.lookup_transform('base',
                                       obj_resp.obj.header.frame_id,  # source frame
                                        rospy.Time(0),  # get the tf at first available time
                                        rospy.Duration(1.0))  # wait for 1 second

pose_transformed = tf2_geometry_msgs.do_transform_pose(obj_resp.obj, transform)
pose = pose_transformed.pose
(r, p, y) = tf.transformations.euler_from_quaternion(
     [pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z,
      pose_transformed.pose.orientation.w])
save_pose = [r,p,y,pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z,
      pose_transformed.pose.orientation.w]

print('pose',save_pose)
np.save('save_pose', save_pose)
# save_pose = np.load('save_pose.npy')
# print('pose',save_pose)
br = tf2_ros.StaticTransformBroadcaster()
tf_send = []
t = geometry_msgs.msg.TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = "base"
t.child_frame_id = "gpis_cloud_pose"
t.transform.translation.x = pose.position.x
t.transform.translation.y = pose.position.y
t.transform.translation.z = pose.position.z
t.transform.rotation.x = pose.orientation.x
t.transform.rotation.y = pose.orientation.y
t.transform.rotation.z = pose.orientation.z
t.transform.rotation.w = pose.orientation.w
tf_send.append(t)
print 'publish the posearray'
br.sendTransform(tf_send)
# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
#     rate.sleep()


new_mean = np.load('new_mean_all.npy')

mean_for_ros_bag = list(new_mean)

import rosbag
from std_msgs.msg import Float32,Float32MultiArray

bag = rosbag.Bag('test.bag', 'w')

try:
    i = Float32()
    i.data = 42

    x = Float32MultiArray()
    for j in range(len(mean_for_ros_bag)):
        x.data.append(mean_for_ros_bag[j])

    bag.write('numbers', i)
    bag.write('floatarray', x)
finally:
    bag.close()
print 'finish writing the rosbag'
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()

'''
def add_pointcloud_to_kdl_tree(urdf):
        root = urdf.get_root()
        tree = kdl.Tree(root)

        def add_children_to_tree(parent):
            if parent in urdf.child_map:
                if parent=='left_gripper_base':
                    child_name_cloud = '1'
                    print('hello')
                    pose = Pose()
                    pose.position.x = 1
                    pose.position.y = 1
                    pose.position.z = 1
                    kdl_origin = point_pose_to_kdl_frame(pose)
                    kdl_jnt = point_joint_to_kdl_joint(child_name_cloud)
                    kdl_sgm = kdl.Segment(child_name_cloud, kdl_jnt,
                                          kdl_origin)
                    tree.addSegment(kdl_sgm, parent)
                    return None
                for joint, child_name in urdf.child_map[parent]:

                    child = urdf.link_map[child_name]
                    if child.inertial is not None:
                        kdl_inert = urdf_inertial_to_kdl_rbi(child.inertial)
                    else:
                        kdl_inert = kdl.RigidBodyInertia()
                    kdl_jnt = urdf_joint_to_kdl_joint(urdf.joint_map[joint])
                    kdl_origin = urdf_pose_to_kdl_frame(urdf.joint_map[joint].origin)
                    kdl_sgm = kdl.Segment(child_name, kdl_jnt, kdl_origin, kdl_inert)
                    tree.addSegment(kdl_sgm, parent)

                    add_children_to_tree(child_name)

        add_children_to_tree(root)
        return tree








for joint, child_name in robot.child_map[root]:
    print(joint,child_name)
    print(robot.joint_map[joint].origin)
    kdl_origin = urdf_pose_to_kdl_frame(robot.joint_map[joint].origin)
    kdl_jnt = urdf_joint_to_kdl_joint(robot.joint_map[joint])
    kdl_sgm = kdl.Segment('hello', kdl_jnt,
                          kdl_origin)
    tree.addSegment(kdl_sgm, root)

    break

joint0=Joint(Joint.RotZ)
frame0=Frame(Vector(1.0,0.0,0.))
segment0=Segment(joint0,frame0)
chain.addSegment(segment0)

k = gp.kern.RBF(input_dim=3, variance=1.0, lengthscale=0.07)

X = np.random.uniform(-0.3,0.3, (2,3))
print('X',X)

Y = np.random.randn(2,1)*0.05
print('Y',Y)

m = gp.models.GPRegression(X, Y, kernel=k)
m.likelihood.variance = 0.1
X_1 = np.random.uniform(-0.3,0.3, (4,3))
print('X_star',X_1)

result = m.predict(X_1,full_cov = True)
mean = result[0]
result = result[1]
slog = np.linalg.slogdet(result)
inv_matrix = np.linalg.inv(result)
transpose_matrix = np.transpose(inv_matrix)
print('mean',mean)

print('inv',inv_matrix)
print('trans',transpose_matrix)
grad = m.predictive_gradients(X_1)
#grad_2 = m.predict_jacobian(X_1)

print('grad',grad[1])
grad = grad[1]
print(np.dot(transpose_matrix,grad))

import GPflow
k_flow = GPflow.kernels.RBF(input_dim=3, variance=1.0,lengthscales=0.07)
meanf = GPflow.mean_functions.Ellipsoidal(0.1,0.1,0.1)
m_flow = GPflow.gpr.GPR(X, Y, kern=k_flow,mean_function=meanf)
m_flow.likelihood.variance = 0.1
var = m_flow.predict_f_full_cov(X_1)
var = var[1]
temp_var = np.zeros((var.shape[0],var.shape[1]),dtype=np.float)
for i in range(var.shape[0]):
    for j in range(var.shape[1]):
        temp_var[i][j] = var[i][j][0]
        if i==j:
            temp_var[i][j] +=0.1
print('gpflow_var',temp_var)
print('gpy_var',result)



limb = 'left'
    #joints_0=np.append(np.array([0.0,0.2,0.0,-1.0,0.0,0.0,1.5]),np.zeros(16))
rs = baxter_interface.RobotEnable(CHECK_VERSION)
limb_interface = baxter_interface.limb.Limb(limb)
current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
current_names = [joint for joint in limb_interface.joint_names()]
angles = baxter_interface.Limb(limb).joint_angles()

print 'current_angles,current_angles'
print angles

test rotating the pose by 90 degrees, and works
'''
#you have to start baxter moveit launch file in order to run this program
if __name__ == '__main__':


    rospy.set_param('robot_description_test', baxter_urdf_path)


    #rospy.spin()
    '''
    #rospy.init_node('dynamic_tf_broadcaster')
    br = tf2_ros.StaticTransformBroadcaster()

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "static_camera1"
    t.transform.translation.x = 1
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    r = 1.57
    p = 0.98
    y = 0
    quat = tf.transformations.quaternion_from_euler(r,p,y)
    print 'quat get',quat
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    #br.sendTransform(t)

    temp = []
    temp.append(t)
    
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
    
    br.sendTransform(temp)
    rospy.spin()
    '''