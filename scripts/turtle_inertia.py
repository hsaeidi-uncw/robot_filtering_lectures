#!/usr/bin/env python3
# import ROS for developing the node
import rospy

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

force = 0.0
vel_cur = float()

# get the force value
def get_force(data):
	global force
	force = data.linear.x 
	
def pose_callback(data):
	global vel_cur
	vel_cur = data.linear_velocity

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
	mass = 1 # m in the equations (kg)
	damping = 0.25 # b in the equations (Ns/m)

	vel_next = 0.0 # initialize the next velocity
	# run this control loop regularly
	while not rospy.is_shutdown():
		vel_next = dt/mass*(force - damping*vel_cur) + vel_cur
		# set the linear (forward/backward) velocity command
		vel_cmd.linear.x = vel_next
		# set the angular (heading) velocity command
		cmd_pub.publish(vel_cmd)
		
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
