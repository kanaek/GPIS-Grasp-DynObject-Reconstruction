#!/usr/bin/env python
# This class publishes the trajectory to rviz for visualization
# TODO: Implement for Allegro hand.
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
import roslib
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal



import numpy as np
_J_NAMES=['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2','l_gripper_l_finger_joint','l_gripper_r_finger_joint']

class trajectoryServer:
    def __init__(self, loop_rate, robot_name='baxter',
                 joint_names=_J_NAMES,
                 topic_name='baxter/joint_trajectory_viz', init_node=False, init_jtas_client=False):
        ## Initialize ros node
        if (init_node):
            rospy.init_node("joint_traj_viz")
        self.rate = rospy.Rate(loop_rate)

        tr_topic = topic_name
        self.pub_tr = rospy.Publisher(tr_topic, DisplayTrajectory, queue_size=1)
        self.joint_names = joint_names
        self.robot_name = robot_name
        if (init_jtas_client):
            self.client = actionlib.SimpleActionClient(topic_name, FollowJointTrajectoryAction)

    def send_traj(self, u_arr):
        # Formulate joint trajectory message:
        jt_msg = JointTrajectory()
        jt_msg.joint_names = self.joint_names
        i = 0
        for i in range(len(u_arr)):
            u = u_arr[i]
            jt_pt = JointTrajectoryPoint()
            jt_pt.positions = u
            jt_pt.time_from_start.secs = i
            i = i + 2
            jt_msg.points.append(jt_pt)

        robot_tr = RobotTrajectory()
        robot_tr.joint_trajectory = jt_msg
        disp_tr = DisplayTrajectory()
        disp_tr.trajectory.append(robot_tr)
        disp_tr.model_id = self.robot_name
        return self.pub_tr,disp_tr
        # self.pub_tr.publish(disp_tr)
        # while (not rospy.is_shutdown()):
        #     self.pub_tr.publish(disp_tr)
        #     self.rate.sleep()

# from joint_trajectory import Trajectory
# import moveit_commander
# import moveit_msgs.msg
# import sys
import itertools

import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl

from sklearn import mixture

color_iter = itertools.cycle(['navy', 'c', 'cornflowerblue', 'gold',
                              'darkorange'])
def plot_results(X, Y_, means, covariances, index, title):
    splot = plt.subplot(2, 1, 1 + index)
    for i, (mean, covar, color) in enumerate(zip(
            means, covariances, color_iter)):
        v, w = linalg.eigh(covar)
        v = 2. * np.sqrt(2.) * np.sqrt(v)
        u = w[0] / linalg.norm(w[0])
        # as the DP will not use every component it has access to
        # unless it needs it, we shouldn't plot the redundant
        # components.
        if not np.any(Y_ == i):
            continue
        plt.scatter(X[Y_ == i, 0], X[Y_ == i, 1], .8, color=color)

        # Plot an ellipse to show the Gaussian component
        angle = np.arctan(u[1] / u[0])
        angle = 180. * angle / np.pi  # convert to degrees
        ell = mpl.patches.Ellipse(mean, v[0], v[1], 180. + angle, color=color)
        ell.set_clip_box(splot.bbox)
        ell.set_alpha(0.5)
        splot.add_artist(ell)

    plt.xlim(-9., 5.)
    plt.ylim(-3., 6.)
    plt.xticks(())
    plt.yticks(())
    plt.title(title)
if __name__=='__main__':
    rospy.init_node("joint_traj_viz")

    n_samples = 1000

    # Generate random sample, two components
    np.random.seed(0)
    C = np.array([[0., -0.1], [1.7, .4]])
    temp = np.dot(np.random.randn(n_samples, 2), C)
    temp2 = .7 * np.random.randn(n_samples, 2) + np.array([-6, 3])
    X = np.r_[temp,
              temp2]

    # Fit a Gaussian mixture with EM using five components
    gmm = mixture.GaussianMixture(n_components=50, covariance_type='full').fit(X)
    plot_results(X, gmm.predict(X), gmm.means_, gmm.covariances_, 0,
                 'Gaussian Mixture')

    print('gmm_weights_',gmm.weights_)
    plt.show()

    # # Fit a Dirichlet process Gaussian mixture using five components
    # dpgmm = mixture.BayesianGaussianMixture(n_components=2,
    #                                         covariance_type='full').fit(X)
    # plot_results(X, dpgmm.predict(X), dpgmm.means_, dpgmm.covariances_, 1,
    #              'Bayesian Gaussian Mixture with a Dirichlet process prior')
    #
    # plt.show()

    # real_angles = [-1.2524953133084402, -0.3601019899561009, -0.04141748127290617, 0.9422476989586154,
    #                -0.06557767868210143, 0.9541360500647273, -1.9907235674782957,0.020833,-0.020833]
    #
    # real_angles_2 = [-1.1524953133084402, -0.3601019899561009, -0.04141748127290617, 0.9422476989586154,
    #                -0.06557767868210143, 0.9541360500647273, -1.9907235674782957,0.020833,-0.020833]
    # real_angles_3 = [-1.5524953133084402, -0.3601019899561009, -0.04141748127290617, 0.9422476989586154,
    #                  -0.06557767868210143, 0.9541360500647273, -1.9907235674782957, 0.020833, -0.020833]
    # pub_traj = [real_angles,real_angles_2,real_angles_3]
    #
    # traj_server = trajectoryServer(10)
    # traj_server.send_traj(pub_traj)
    # print
    # "============ Starting tutorial setup"
    # moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('move_group_python_interface_tutorial',
    #                 anonymous=True)
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group = moveit_commander.MoveGroupCommander("left_arm")
    # print "============ Reference framefffffff: %s" % group.get_end_effector_link()
    # print "============ Robot Groups:"
    # print robot.get_group_names()