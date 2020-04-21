#!/usr/bin/env python
# coding=utf-8

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import LaserScan


class ScanMatching:
    def __init__(self):
        rospy.init_node('scan_matcher')

        self.MAX_ITERATIONS = 100  # maximum iterations for running your scan-matching for convergence
        self.estimated_pose = PoseStamped()

        # Publisher for estimated pose of car obtained from scan matching
        self.pose_pub = rospy.Publisher('/scan_match_pose', PoseStamped, queue_size=1)
        # Publisher for 'drive_parameters' (speed and steering angle)
        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=1)
        # Publisher for publishing laserscan points of previous frame transformed by current estimate of transform.
        self.fake_scan_publisher = rospy.Publisher('fake_scan_match', MarkerArray, queue_size="1")
        # Subscriber for odometry and laserscan messages
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

    def odom_callback(self, msg):
        '''get odometry information from odometry topic'''

    def scan_callback(self, msg):
        '''get laserscan data from the scan topic'''

    def do_scan_matching(self):
        rospy.loginfo("Scan matching started.")
        # The following steps are inspired from the Censi's PL-ICP paper. Try to modularize your code for each step to make it more readable and debuggable.
        '''1. Compute the coordinates of the second scan’s points in the first scan’s frame of reference, 
         according to the roto-translation obtained from odometry update.'''

        '''2.Find correspondence between points of the current and previous frame. You can use naive way of looking 
        through all points in sequence or use radial ordering of laser points to speed up the search.'''

        '''3. Based on the correspondences, find the necessary tranform.'''
        # 3.a. Construct the necessary matrices as shown in the paper for solution with Lagrange's multipliers.

        # 3.b. You should get a fourth order polynomial in lamda which you can solve to get value(hint:greatest real root of polynomial equn) of lamda.

        # 3.b. Use the calcualted value of lamda to estimate the transform using equation 24 in the Censi's paper.

        '''4.Publish the estimated pose from scan matching based on the transform obstained. You can visualize the pose in rviz.'''

        '''5.Also transform the previous frame laserscan points using the roto-translation transform obtained and visualize it. Ideally, this should coincide
        with your actual current laserscan message.'''


if __name__ == '__main__':
    scan_matching = ScanMatching()
    scan_matching.do_scan_matching()
    rospy.spin()

