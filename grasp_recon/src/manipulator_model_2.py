#!/usr/bin/env python

import rospy
import roslib
from urdf_parser_py.urdf import Robot
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
#from sensor_msgs.msg import JointState

import numpy as np
from math import sin, cos, pi
import time

_DISPLAY_RATE = 100
_EE_NAME ='right_gripper'
_J_NAMES=['torso_t0','right_torso_arm_mount','right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2','right_endpoint']
_LINK_NAMES=['base','torso','right_arm_mount','right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_upper_forearm','right_lower_forearm',
             'right_wrist','right_hand','right_gripper_base','right_gripper']


_TEST_TORQUE_CONTROL=False
_TEST_IK_CONTROL=True

class ManipulatorSimpleModel:
    def __init__(self, urdf_file_name,base_link="base_link",ee_name=_EE_NAME):
        '''
        Simple model of manipulator kinematics and controls
        Assume following state and action vectors
        urdf_file_name - model file to load
        '''
        # Load KDL tree
        urdf_file = file(urdf_file_name, 'r')
        self.robot = Robot.from_xml_string(urdf_file.read())
        urdf_file.close()
        self.tree = kdl_tree_from_urdf_model(self.robot)
       
        task_space_ik_weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0]).tolist()

        #self.base_link = self.robot.get_root()
        self.base_link = base_link
        
        self.joint_chains=[]

        self.chain = KDLKinematics(self.robot, self.base_link, ee_name)


        for l_name in _LINK_NAMES:
            jc=KDLKinematics(self.robot,self.base_link,l_name)
            self.joint_chains.append(jc)



    def FK_joint(self,joint_angles,j_index):
        '''
        Method to return task coordinates between base link and any joint
        joint_angles must contain only 0:j_index joints
        '''
        fi_x=self.joint_chains[j_index].forward(joint_angles)

        return fi_x

    def Jacobian_joint(self,joint_angles,j_index):
        ji_x=self.joint_chains[j_index].jacobian(joint_angles)
        return ji_x
    
    def FK(self, joint_angles):
        '''
        Method to convert joint positions to task coordinates
        '''

        fi_x = self.chain.forward(joint_angles)
        return fi_x
    
    def Jacobian(self,joint_angles):
        ji_x = self.chain.jacobian(joint_angles)
        return ji_x


    #TODO: CHECK if IK works
    def IK(self, fingers_desired, finger=None):
        '''
        Get inverse kinematics for desired finger poses of all fingers
        fingers_desired - a list of desired finger tip poses in order of finger_chains[]
        returns - a list of lists of pyhton arrays of finger joint configurations
        TODO: Allow for single finger computation (named fingers)
        '''
        q_desired_fingers = []
        if finger is not None:
            # TODO: solve for a single finger...
            return None
        for i, f_d in enumerate(fingers_desired):
            q_desired_fingers.append(self.finger_chains[i].inverse_wdls(f_d))
            # q_desired_fingers.append(self.finger_chains[i].inverse(f_d))
        return q_desired_fingers
