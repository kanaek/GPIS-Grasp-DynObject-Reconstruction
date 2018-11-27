#!/usr/bin/env python
# This server listens to a desired end-effector pose and computes a joint trajectory:
import rospy
from optimization_baxter.srv import *
from pcl_mesh_tools.srv import update_env
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from tabletop_obj_segmentation.srv import SegmentGraspObject
import sys
import time

from rospkg import RosPack
rp=RosPack()
rp.list()
path=rp.get_path('ll4ma_kdl')+'/scripts'

sys.path.insert(0,path)
from kdl_lbr4_model import *

from trajectory_pub import *

def get_plan():
    rospy.wait_for_service('grasp_planner')
    ik_plan=rospy.ServiceProxy('grasp_planner',IKJointTrajectory)
    #joints_des=np.array([c-0.8,-1.0,0.0,0.3,0.0,-0.8,0.5])
    joints_0=np.append(np.array([0.0,0.2,0.0,-1.0,0.0,0.0,1.5]),np.zeros(16))


    # Get object centroid pose from object segmentation service:
    rospy.wait_for_service('object_segmenter')
    obj_pose_srv=rospy.ServiceProxy('object_segmenter',SegmentGraspObject)
    obj_resp=obj_pose_srv(False)
    pose=obj_resp.obj.pose
    x_des=np.zeros(6)
    
    # Position:
    x_des[0]=pose.position.x
    x_des[1]=pose.position.y
    x_des[2]=pose.position.z
    start = time.time()
    print x_des
    resp=ik_plan(x_des,joints_0)
    end = time.time()
    print 'Tjo took: '+str(end - start)+' seconds'
    return resp.joint_traj



if __name__=='__main__':
    rospy.init_node("grasp_client")
    # Update collision server
    # Call collision initialization service:
    rospy.wait_for_service('collision_checker/update_env_cloud')
    update_env_cloud=rospy.ServiceProxy('collision_checker/update_env_cloud',update_env)
    res=update_env_cloud()
    
    print res.init_env
    #Wait for grasp services:
    rospy.wait_for_service('collision_checker')
    rospy.wait_for_service('grasp/contact_checker')

    print "All services online"
    # Get plan
    j_traj=get_plan()

    # Send joint trajectory to visualizer
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

