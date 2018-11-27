#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose,PoseArray,Point
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
print 'hello'
point_center = Point()
point_center.x = 0.0
l_pose = Pose()
l_pose.position.x = 0
l_pose.position.y = 1
l_pose.position.z = 2
l_pose.orientation.x = 0
l_pose.orientation.y = 1
l_pose.orientation.z = 0
l_pose.orientation.w = 0

pose_array = PoseArray()
pose_array.header.frame_id = 'camera_link' #for real application, should change to 'base'
pose_array.poses.append(l_pose)

l_pose = Pose()
l_pose.position.x = 1
l_pose.position.y = 1
l_pose.position.z = 2
l_pose.orientation.x = 0
l_pose.orientation.y = 1
l_pose.orientation.z = 0
l_pose.orientation.w = 0

pose_array.poses.append(l_pose)





if __name__ == '__main__':
    rospy.init_node("rsdk_joint_trajectory_client")
    print 'ddd'
    pub = rospy.Publisher('posearray', PoseArray, queue_size=10)
    rate = rospy.Rate(10)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    current = left.position()
    print 'current',current
    '''
    while not rospy.is_shutdown():
        pub.publish(pose_array)
        rate.sleep()
    '''