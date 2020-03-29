#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math

# TODO: modify these constants to make the car follow walls smoothly.
KP = 0.0
KD = 0.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(msg):
  # TODO: Based on the error (msg.data), determine the car's required velocity
  # amd steering angle.
  	pid_output = 0  #TODO: compute pid response for steering angle based on the error

  	vel = #TODO: compute velocity based on the steering angle response. Remember you need to slow down accordingly based on how much you turn.	

  	angle = math.radians(pidoutput)    #convert the angle to radians if not already in radians
	angle = np.clip(angle, -0.4189, 0.4189)  # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
	
	msg = drive_param()
	msg.velocity = vel  # TODO: implement PID for velocity
	msg.angle = angle    # TODO: implement PID for steering angle
	pub.publish(msg)

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", Float64, control_callback)
	rospy.spin()

