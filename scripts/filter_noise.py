#!/usr/bin/env python3
import rospy
# we are going to read turtlesim/Pose messages this time
from turtlesim.msg import Pose

pos_msg = Pose()
 
pos_received = False

def pose_callback(data):
	global pos_msg
	global pos_received 
	# update the current message
	pos_msg = data
	pos_received = True

	
if __name__ == '__main__':
	# initialize the node
	rospy.init_node('filter_noise', anonymous = True)
	# add a subscriber to it to read the position information
	pos_sub = rospy.Subscriber('/turtle1/noisy_pose', Pose, pose_callback)
	# add a publisher for publishing the filtered message
	pos_pub = rospy.Publisher('/turtle1/filtered_pose', Pose, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	fil_in = 0.0 # before the first reading
	fil_out = 5.5 # an initial guess before the first reading
	fil_gain = 0.05 # how much of the most recent input is included 
	while not rospy.is_shutdown():
		if pos_received:
			# update the filter input
			fil_in = pos_msg.x
			print('fil_in:', fil_in, ' fil_out now:', fil_out) 
			# filter the position
			fil_out = fil_gain*fil_in + (1 - fil_gain)*fil_out
			print('fil_out_next:', fil_out) 
			# update and publish the message
			pos_msg.x = fil_out
			pos_pub.publish(pos_msg)
			# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
