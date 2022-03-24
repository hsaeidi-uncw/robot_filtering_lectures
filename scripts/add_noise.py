#!/usr/bin/env python3
import rospy
# we are going to read turtlesim/Pose messages this time
from turtlesim.msg import Pose
# for generating random noise
import random


pos_msg = Pose()

def pose_callback(data):
	global pos_msg
	# update the current message
	pos_msg = data
	# add some noise to the x coordinate
	noise_amplitude = 0.25 # in meters
	pos_msg.x += random.uniform(-noise_amplitude, noise_amplitude)
	
	
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('add_noise', anonymous = True)
	# add a subscriber to it to read the position information
	pos_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	# add a publisher for the noisy position data
	pos_pub = rospy.Publisher('/turtle1/noisy_pose', Pose, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# publish the message
		pos_pub.publish(pos_msg)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
