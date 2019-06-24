#!/usr/bin/env python

# testing code for the multi-bot simulation, mainly that i can move them independently

import rospy
import numpy as np

from std_msgs.msg import Header
from uuv_control_msgs.srv import GoTo
from uuv_control_msgs.msg import Waypoint
from geometry_msgs.msg import Point, Vector3, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


#CONSTANTS
rov1_startx = 100
rov2_startx = 80
starty = 20 


#Global Vars
auv1_location = None             #global var for robot position
auv1_heading = None              #global var for robot heading vector
auv2_location = None
auv2_heading = None

#see musa's example waypoint for eca a9 in discord
#Waypoint messsage 'constructor'
def make_waypoint(newx,newy,newz):
	#create waypoint message
	wp = Waypoint()
	wp.header.stamp = rospy.Time.now()
	wp.header.frame_id = "world"
	wp.point.x = newx
	wp.point.y = newy
	wp.point.z = newz
	wp.max_forward_speed = 2.0
	wp.heading_offset = 0.0
	wp.use_fixed_heading = False
	wp.radius_of_acceptance = 0.5
	return wp


#Go To service call
def call_goto(wp, gotoservice, interpolator):
	#rosservice call to Go_To
	try:
		res = gotoservice(wp,wp.max_forward_speed,str(interpolator))
		#print("Go To service call successful: " + str(res))
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)


#callback function for auv pose subscriber
def readauv1pose(msg):
	global auv1_heading
	auv1_heading = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])


def readauv2pose(msg):
	global auv2_heading
	auv2_heading = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])


if __name__=='__main__':
	rospy.init_node('multi_bot_test')

	auv1pos_sub = rospy.Subscriber(
		'rov1/pose_gt',
		Odometry,
		readauv1pose)

	auv2pos_sub = rospy.Subscriber(
		'rov2/pose_gt',
		Odometry,
		readauv2pose)
	
	interpolator = rospy.get_param('~interpolator', 'dubins')
	
	try:
		rospy.wait_for_service('rov1/go_to', timeout=15)
	except rospy.ROSException:
		raise rospy.ROSException('rov1 Service not available!')

	try:
		rospy.wait_for_service('rov2/go_to', timeout=15)
	except rospy.ROSException:
		raise rospy.ROSException('rov2 Service not available!')
	
	try:
		goto1 = rospy.ServiceProxy('rov1/go_to', GoTo)
	except rospy.ROSException as e:
		raise rospy.ROSException('rov1 service proxy failed, error=%s', str(e))

	try:
		goto2 = rospy.ServiceProxy('rov2/go_to', GoTo)
	except rospy.ROSException as e:
		raise rospy.ROSException('rov2 service proxy failed, error=%s', str(e))

	while not rospy.is_shutdown():
	
		r = rospy.Rate(1)
		r.sleep()

		r1wp1 = make_waypoint(rov1_startx,starty,-31)
		r2wp1 = make_waypoint(rov2_startx,starty,-31)
		call_goto(r1wp1,goto1,interpolator)
		call_goto(r2wp1,goto2,interpolator)