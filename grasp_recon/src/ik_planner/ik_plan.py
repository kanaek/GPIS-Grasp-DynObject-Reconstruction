#!/usr/bin/env python
import rospy
import numpy as np
from trac_ik_baxter.srv import *
from geometry_msgs.msg import PoseStamped,Pose
from sensor_msgs.msg import JointState
import baxter_interface
from baxter_pykdl import baxter_kinematics
class ik_solver:
    def __init__(self,srv_name='trac_ik_left',end_tolerance = 5, num_steps = 100):
        # Wait for service
        self.end_tolerance = end_tolerance
        self.num_steps = num_steps
        print 'wait'
        print "try to find trac_ik_left service"
        rospy.wait_for_service(srv_name)
        self.ik=rospy.ServiceProxy(srv_name,GetConstrainedPositionIK,persistent=True)
        print "Found trac_ik_left service"
    def ik_solve(self,obj_pose,current_angles):
        
        header = 'base'

        pose_array = []
        seed_angles = []
        pose_stamp = PoseStamped()
        pose_stamp.header.frame_id = header
        pose_stamp.pose = obj_pose
        '''
        pose_stamp.pose.position.x = obj_pose[0]
        pose_stamp.pose.position.y = obj_pose[1]
        pose_stamp.pose.position.z = obj_pose[2]
        pose_stamp.pose.orientation.x = obj_pose[3]
        pose_stamp.pose.orientation.y = obj_pose[4]
        pose_stamp.pose.orientation.z = obj_pose[5]
        pose_stamp.pose.orientation.w = obj_pose[6]
        '''
        pose_array.append(pose_stamp)
        seed_angles.append(current_angles)
        try:
            # Call collision checker:
            resp=self.ik(pose_array,seed_angles,self.end_tolerance,self.num_steps)
            #print resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            #exit()
        solution=[]
        print 'isvalid',resp.isValid
        for i in range(len(resp.joints)):
            for j in resp.joints[i].position:
              solution.append(j)
        # return solution
        return solution,resp.isValid

'''
def test_client():
    c_checker=ik_solver()
    #r_poses=[np.zeros(7) for i in range(7)]
    #r_data=c_checker.get_signed_distance(r_poses)
    #print r_data
    
if __name__ == '__main__':
    rospy.init_node("ik_server")
    temp = [0.84402854,-0.02152282,-0.07460576,0.39870879,0.89748961,0.05885463,0.17910843]


    l_pose = Pose()
    obj_pose =  []
    l_pose.position.x = temp[0]
    l_pose.position.y = temp[1]
    l_pose.position.z = temp[2]
    l_pose.orientation.x = temp[3]
    l_pose.orientation.y = temp[4]
    l_pose.orientation.z = temp[5]
    l_pose.orientation.w = temp[6]
    obj_pose.append(l_pose)
    limb = baxter_interface.Limb('left')
    angles = limb.joint_angles()
    joint_angle = JointState()
    for i in angles:
        joint_angle.name.append(i)
        joint_angle.position.append(angles[i])
    c_checker = ik_solver()
    resp = c_checker.ik_solve(l_pose,joint_angle)
    print resp.isValid
    print resp.joints
    print resp.accepted_tolerance

    kin = baxter_kinematics('left')
    print 'hello'
    print kin.forward_position_kinematics()
    #r_poses=lbr4.allegro.collision_link_poses([-0.44830588525951215])
    #print r_poses
    print 'hello'
    #rospy.spin()
'''
