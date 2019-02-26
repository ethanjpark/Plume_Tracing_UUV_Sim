#!/usr/bin/env python

# Chemical Plume Tracing - Behaviour Based Planning Algorithm

# Algorithm based on: 
# "Chemical Plume Tracing via an Autonomous Underwater Vehicle" by 
# Jay A. Farrell, Shuo Pang, and Wei Li

import rospy
import numpy as np

from uuv_sensor_ros_plugins_msgs.msg import ChemicalParticleConcentration
from geometry_msgs.msg import Vector3, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry

THRESHOLD = 0.1 #particle concentration threshold for detecting plume
CURRENT_FLOW = np.array([1.0, 0.0]) # [x,y] vector of the current flow
BETA_OFFSET = 20 #angle offset relative to upflow

particle_concentration #global var for particle concentration
auv_location #global var for robot position
auv_heading #global var for robot heading

#Track In behavior of algorithm
def track_in():
	if(particle_concentration >= THRESHOLD):
		#calculate LHS binary var
		###############
		#LEFT OFF HERE#
		###############

#callback function for particle concentration subscriber
def readconcentration(msg):
	global particle_concentration, auv_location

	particle_concentration = msg.concentration
	auv_location = np.array([msg.position.x, msg.position.y, msg.position.z])


#callback function for auv pose subscriber
def readauvpose(msg):
	global auv_heading

	auv_heading = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y])


def main():
	rospy.init_node('CPT_BBP')

	part_conc_sub = rospy.Subscriber(
		'rexrov2/particle_concentration', 
		ChemicalParticleConcentration, 
		readconcentration)

	auvpos_sub = rospy.Subscriber(
		'rexrov2/pose_gt',
		Odometry,
		readauvpose)

	rospy.spin()


if __name__=='__main__':
	main()