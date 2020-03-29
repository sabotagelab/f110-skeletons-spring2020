#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np

#this publishes the drive command to run the car.
drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

#this publishes the center of the gap in(x,y,z) coordinate system
gap_center_pub = rospy.Publisher("gap_center", Vector3, queue_size=1)

#Implement your own message to publish relevant information about gaps
gaps_pub = rospy.Publisher("gaps", <your message type>, queue_size=1)

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):
	#Implement some clustering algorithm(e.g K-means,DBSCAN,your own custom algorithm) to cluster points and find possible gaps

    #Publish the center of the gaps as well as the endpoints of the gaps.

    #Based on the center of the gap, you can also implement a simple PID controller to follow it. Be careful about the lookahead distance you use.
	msg = drive_param()
	msg.velocity = 0.1  # TODO: implement PID for velocity
	msg.angle = 0.0    # TODO: implement PID for steering angle
	drive_pub.publish(msg)


# Boilerplate code to start this ROS node.
if __name__ == '__main__':
	rospy.init_node('gap_finding_node', anonymous=True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

