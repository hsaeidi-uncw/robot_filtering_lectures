#!/usr/bin/env python3
# import ROS for developing the node
import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# for reading the force commands
force = 0.0

pos_cur = float()

key_released = True # by default we assume that they arrow key on keyboard is not pressed
 
# get the force value
def get_force(data):
	global force
	global key_released
	force = data.linear.x # update the force value 
	key_released = False # now the key is pressed
	
# read the position value
def pose_callback(data):
	global pos_cur
	pos_cur = data.x

if __name__ == '__main__':

	# initialize the node
	rospy.init_node('turtle_inertia', anonymous = True)
	# declare a publisher to publish in the velocity command topic
	cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	# define the force command subscriber
	force_sub = rospy.Subscriber("/turtle1/cmd_force", Twist, get_force) 
	# add a subscriber to it to read the position information
	pos_sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	# set a 10Hz frequency for this loop
	freq = 10
	loop_rate = rospy.Rate(freq)
	# define the delta t 
	dt = 1/freq
	
	# declare a variable of type Twist for sending control commands
	vel_cmd = Twist()
	# define the paremeters of the equations 
	mass = 1.0 # m in the equations (kg)
	damping = 1.0 # b in the equations (Ns/m)
	vel_next = 0.0 # initialize the next velocity
	pos_prev = 0.0 # initialize the previous position
	# run this control loop regularly
	while not rospy.is_shutdown():
		print(key_released)
		if key_released:
			force = 0.0 # make sure to remove non-zero commands when key is released
		vel_cur = (pos_cur - pos_prev)/dt
		vel_next = dt/mass*(force - damping*vel_cur) + vel_cur
		print('force = ', force, ' (N) and linear velocity=', vel_cur, 'm/s', 'and next velocity=', vel_next, 'm/s')
		# set the linear (forward/backward) velocity command
		vel_cmd.linear.x = vel_next
		cmd_pub.publish(vel_cmd)
		# update the pose for next iteration
		pos_prev = pos_cur
		key_released = True # assume the key is not pressed unless we receive a new command which proves otherwise
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
