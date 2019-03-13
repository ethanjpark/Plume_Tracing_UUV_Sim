#!/usr/bin/env python

# written by: ETHAN PARK

# Chemical Plume Tracing - Behaviour Based Planning Algorithm

# Algorithm based on: 
# "Chemical Plume Tracing via an Autonomous Underwater Vehicle" by 
# Jay A. Farrell, Shuo Pang, and Wei Li

import rospy
import numpy as np

from std_msgs.msg import Header
from uuv_control_msgs.srv import GoTo
from uuv_control_msgs.msg import Waypoint
from uuv_sensor_ros_plugins_msgs.msg import ChemicalParticleConcentration
from geometry_msgs.msg import Point, Vector3, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

#CONSTANTS
THRESHOLD = 0.004 					#particle concentration threshold for detecting plume
CURRENT_FLOW = np.array([1.0, -1.0]) #[x,y] vector of the current flow
BETA_OFFSET = 30 					#angle offset relative to upflow
UPFLOW = np.array([-1.0, 1.0]) 		#180 rotation of CURRENT_FLOW
LAMBDA = 2.0 						#plume detection time threshold (2 seconds)
R = 0.75								#distance threshold to find new ldp waypoint
L_u = 2.0							#constant for how much upflow from last detected location auv should go
L_c = 2.0							#constant for how much cross flow from last detected location auv should go
startx = 20							#x-component of where auv should start from
starty = 25							#y-component of where auv should start from

#Global Vars
alg_state = -1					#global var for which state the algorithm is currently in
								# 0 for init, 1 for find, 2 for track-in, 3 for track-out, 4 for reacquire, 5 (maybe) for source declared
particle_concentration = 0.0    #global var for particle concentration
auv_location = None             #global var for robot position
auv_heading = None              #global var for robot heading vector
lhs = 0                     	#global var for which side of plume robot will drive out of
t_last = 0                  	#global var for last time at which plume was detected
lost_pnts = []          		#last detection points stored when track out is triggered
ldp = None                     	#global var for last detection point
tout_init = 1					#global var indicating whether track-out needs to choose the next upflow last detected point
tout_wp = None					#global var for storing waypoint for track-out behavior
bowtie_step = -1				#global var for which step of the bowtie maneuver auv is currently performing
								# 0 = going to center, 1 for upflow left, 2 for downflow left, 3 for upflow right, 4 for downflow right
upnotcross = 1					#global var indicator for when auv is going upflow not cross (for hitting boundary)
findpos = -1					#global var for indicating which direction of rotation from upflow auv is going
findbound = 0					#global var indicating which boundary (pos/neg x or pos/neg y) the auv hit
								# 1 for pos x, 2 for neg x, 3 for pos y, 4 for neg y
prevfindbound = 0				#global var to keep track of which edge we hit last
trackincounter = 3				#global var used to only periodically call goto in trackin

#dictionary for mapping alg_state to behaviors
s2b = {
	-1: 'Init',
	0 : 'GoTo',
	1 : 'Find',
	2 : 'Track-In',
	3 : 'Track-Out',
	4 : 'Reacquire',
	5 : 'Source Declared'
}

#Calculate angle between two vectors (counter-clockwise positive)
def angle_between(v1,v2):
	dot = np.dot(v1, v2)
	det = np.linalg.det(np.array([v1, v2]))
	return np.rad2deg(np.arctan2(det,dot))

#Calculate normalized rotated vector of upflow
def rotate_upflow(angle):
	rotmatrix = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
	temp = np.array([[UPFLOW[0]],[UPFLOW[1]]])
	new_heading = np.dot(rotmatrix, temp) #2D heading
	new_heading = new_heading/np.linalg.norm(new_heading)
	return new_heading

#Waypoint messsage 'constructor'
def make_waypoint(newx,newy,newz):
	#create waypoint message
	wp = Waypoint()
	wp.header.stamp = rospy.Time.now()
	wp.header.frame_id = "world"
	wp.point.x = newx
	wp.point.y = newy
	wp.point.z = newz
	wp.max_forward_speed = 0.75
	wp.heading_offset = 0.0
	wp.use_fixed_heading = False
	return wp

#Go To service call
def call_goto(wp, gotoservice, interpolator):
	#rosservice call to Go_To
	try:
		res = gotoservice(wp,wp.max_forward_speed,str(interpolator))
		#print("Go To service call successful: " + str(res))
	except rospy.ServiceException, e:
		print("Service call failed: %s"%e)

#Check distance between two locations
def has_reached(a, b, thres):
	return (np.linalg.norm(a-b) < thres)

#Check auv location for boundaries
def check_bounds(location):
	global findbound

	if(location[0] > 100):
		findbound = 1
		return False
	elif(location[0] < -100):
		findbound = 2
		return False
	elif(location[1] > 50):
		findbound = 3
		return False
	elif(location[1] < -50):
		findbound = 4
		return False
	elif(location[2] < -50 or location[2] > 0): #shouldn't really ever trigger but for redundancy
		return False
	else:
		return True

#Track In behavior of algorithm
def track_in(gotoservice,interpolator):
	global lhs, t_last, ldp, alg_state, trackincounter

	trackincounter += 1

	if(not check_bounds(auv_location)): #hit boundary, reflect
		print("track in hit boundary")
		if(lhs == 1):
			lhs = -1
		elif(lhs == -1):
			lhs = 1

		#update t_last
		t_last = rospy.get_time()

		#update last detection point
		ldp = auv_location

		#calculate heading and new waypoint
		offsetrad = lhs*np.deg2rad(BETA_OFFSET)
		new_heading = rotate_upflow(offsetrad)
		threed_heading = np.array([new_heading[0],new_heading[1],0.0])
		new_waypoint = np.add(threed_heading,auv_location)
		
		wp = make_waypoint(new_waypoint[0], new_waypoint[1], new_waypoint[2])
		call_goto(wp, gotoservice, interpolator)

	elif(particle_concentration >= THRESHOLD): #stay in track-in
		alg_state = 2
		if(lhs == 0):
			#calculate lhs var using angle between upflow and auv_heading
			ang = angle_between(UPFLOW,auv_heading)
			print("AUV heading: (" + str(auv_heading[0]) + "," + str(auv_heading[1]) + ")")
			print("Angle between: " + str(ang))
			if(ang > 0): #heading is counter-clockwise from upflow
				lhs = 1
			else:
				lhs = -1

		if(trackincounter%4 == 0):
			#update t_last
			t_last = rospy.get_time()

			#update last detection point
			ldp = auv_location

			#calculate heading and new waypoint
			print("lhs: " + str(lhs))
			offsetrad = lhs*np.deg2rad(BETA_OFFSET)
			new_heading = np.dot(2,rotate_upflow(offsetrad))
			threed_heading = np.array([new_heading[0],new_heading[1],0.0])
			new_waypoint = np.add(threed_heading,auv_location)
			
			wp = make_waypoint(new_waypoint[0], new_waypoint[1], new_waypoint[2])
			call_goto(wp, gotoservice, interpolator)

	#lost contact with plume
	elif(rospy.get_time() - t_last > LAMBDA): #go to track-out
		lost_pnts.append(ldp)
		print("Lost contact with plume, going to track-out.")
		alg_state = 3
		lhs = 0

#Track out behavior of algorithm
def track_out(gotoservice,interpolator):
	global tout_init, alg_state, tout_wp

	#plume detected again
	if(particle_concentration >= THRESHOLD):
		tout_init = 1
		S = src_check()
		if(S):
			alg_state = 5 #source has been found
		else:
			print("Plume found, going to track-in.")
			alg_state = 2 #back to track in

	else:
		if(tout_init == 1): #determine destination waypoint
			up = np.array([lost_pnts[-1][0], lost_pnts[-1][1]]) #set to most upflow point in last detection point list
			tout_init = 0
			#set destination to a point that is upflow and cross the flow from last detected point
			f_p = np.ndarray.flatten(rotate_upflow(np.pi/2))
			f = UPFLOW/np.linalg.norm(UPFLOW)
			tout_wp = up - np.dot(L_u,f) - np.dot(L_c*lhs,f_p)
			wp = make_waypoint(tout_wp[0], tout_wp[1], lost_pnts[-1][2])
			print("Going somewhere based on ldp.")
			call_goto(wp, gotoservice, interpolator)

		#has gotten close enough to designated ldp waypoint
		if(has_reached(auv_location, np.array([tout_wp[0], tout_wp[1], auv_location[2]]), R)):
			tout_init = 1
			S = src_check()
			if(S):
				alg_state = 5 #source has been found
			else:
				print("Going to reacquire.")
				alg_state = 4 #go to reacquire
		#go to ldp waypoint
		# else:
		# 	wp = make_waypoint(tout_wp[0], tout_wp[1], lost_pnts[-1][2])
		# 	print("Going somewhere based on ldp.")
		# 	call_goto(wp, gotoservice, interpolator)

#function for checking whether source can be determined from ldp list
def src_check():
	if(len(lost_pnts) < 3): #not enough data to make conclusion
		print("Not enough data to determine source.")
		return False
	else:
		v1 = np.array([lost_pnts[-3][0]-lost_pnts[-1][0], lost_pnts[-3][1]-lost_pnts[-1][1]]) #vector from 3rd to 1st point (in terms of how upflow)
		v2 = np.array([lost_pnts[-2][0]-lost_pnts[-1][0], lost_pnts[-2][1]-lost_pnts[-1][1]])	#vector from 2nd to 1st point (in terms of how upflow)
		v3 = np.array([lost_pnts[-3][0]-lost_pnts[-2][0], lost_pnts[-3][1]-lost_pnts[-2][1]]) #vector from 3rd to 2nd point (in terms of how upflow)
		#calculate scalar projection of vectors onto upflow vector
		temp = np.linalg.norm(UPFLOW)
		print("Calculating distances between three most upflow points in direction of upflow...")
		p1 = np.dot(UPFLOW, v1)/temp
		p2 = np.dot(UPFLOW, v2)/temp
		p3 = np.dot(UPFLOW, v3)/temp

		if(p1 < 4 and p2 < 4 and p3 < 4):
			print("Source determined!")
			return True
		else:
			print("Data inconclusive, source cannot be determined with accuracy.")
			return False

#Reacquire behavior of algorithm
def reacquire(gotoservice, interpolator):
	global bowtie_step, alg_state, lost_pnts

	if(particle_concentration >= THRESHOLD):
		print("Plume found, going to track-in.")
		bowtie_step = -1
		alg_state = 2
	else:
		#calculate vertices of bowtie maneuver
		bowtie_center = lost_pnts[-1] #most upflow ldp is center of bowtie maneuver
		angle1 = np.deg2rad(15)
		angle2 = np.deg2rad(165)
		angle3 = np.deg2rad(-15)
		angle4 = np.deg2rad(-165)
		uleft = 5*rotate_upflow(angle1) #multiplied by 2 for a bigger maneuver since output of rotate_upflow is normalized
		dleft = 5*rotate_upflow(angle2)
		uright = 5*rotate_upflow(angle3)
		dright = 5*rotate_upflow(angle4)
		bowtie_uleft = np.array([auv_location[0]+uleft[0], auv_location[1]+uleft[1], auv_location[2]])
		bowtie_dleft = np.array([auv_location[0]+dleft[0], auv_location[1]+dleft[1], auv_location[2]])
		bowtie_uright = np.array([auv_location[0]+uright[0], auv_location[1]+uright[1], auv_location[2]])
		bowtie_dright = np.array([auv_location[0]+dright[0], auv_location[1]+dright[1], auv_location[2]])

		if(bowtie_step == -1):	#go to center of bowtie
			bowtie_step = 0
			wp = make_waypoint(bowtie_center[0], bowtie_center[1], bowtie_center[2])
			print("Going to center of bowtie.")
			call_goto(wp, gotoservice, interpolator)
		elif(bowtie_step == 0):	#check if center reached, if so then start bowtie
			if(has_reached(auv_location, bowtie_center, R)):
				bowtie_step = 1
				wp = make_waypoint(bowtie_uleft[0], bowtie_uleft[1], bowtie_uleft[2])
				print("Going to upper left of bowtie.")
				call_goto(wp, gotoservice, interpolator)
		elif(bowtie_step == 1):
			if(has_reached(auv_location, bowtie_uleft, R)):
				bowtie_step = 2
				wp = make_waypoint(bowtie_dleft[0], bowtie_dleft[1], bowtie_dleft[2])
				print("Going to lower left of bowtie.")
				call_goto(wp, gotoservice, interpolator)
		elif(bowtie_step == 2):
			if(has_reached(auv_location, bowtie_dleft, R)):
				bowtie_step = 3
				wp = make_waypoint(bowtie_uright[0], bowtie_uright[1], bowtie_uright[2])
				print("Going to upper right of bowtie.")
				call_goto(wp, gotoservice, interpolator)
		elif(bowtie_step == 3):
			if(has_reached(auv_location, bowtie_uright, R)):
				bowtie_step = 4
				wp = make_waypoint(bowtie_dright[0], bowtie_dright[1], bowtie_dright[2])
				print("Going to lower right of bowtie.")
				call_goto(wp, gotoservice, interpolator)
		elif(bowtie_step == 4):
			if(has_reached(auv_location, bowtie_dright, R)): #end of bowtie maneuver reached without finding plume
				lost_pnts = lost_pnts[:-1]	#remove most upflow point and start again
				if(len(lost_pnts) == 0): #no more ldp points to go through
					print("Couldn't find plume after bowtie, going to find.")
					find_plume(gotoservice, interpolator)
				else:
					bowtie_step = -1

#Find behavior in algorithm
def find_plume(gotoservice, interpolator):
	global alg_state, upnotcross, findpos, prevfindbound

	if(particle_concentration >= THRESHOLD):
		print("Plume found, going to track-in.")
		alg_state = 2
	else:
		alg_state = 1
		if(upnotcross == 1):
			print("find: going across")
			upnotcross = 0
			if(findpos == 1):
				findpos = -1
			elif(findpos == -1):
				findpos = 1
			cross = rotate_upflow(findpos*np.pi/2)
			done = False
			while(not done):
				cross += cross
				if(auv_location[0]+cross[0] < -100 or auv_location[0]+cross[0] > 100 or auv_location[1]+cross[1] < -50 or auv_location[1]+cross[1] > 50):
					done = True
			wp = make_waypoint(auv_location[0]+cross[0], auv_location[1]+cross[1], auv_location[2])
			call_goto(wp, gotoservice, interpolator)
		else:
			if(not check_bounds(auv_location) and prevfindbound != findbound): #hit boundary, go upflow slightly before crossing
				print("find: hit boundary")
				upnotcross = 1
				ufnorm = UPFLOW/np.linalg.norm(UPFLOW)
				#depending on which boundary the auv hit, have to adjust the vector so it doesn't keep going out of bounds
				if(findbound == 1): #triggered on positive x boundary
					prevfindbound = 1
					if(ufnorm[0] > 0): ufnorm[0] = 0
				elif(findbound == 2): #triggered on negative x boundary
					prevfindbound = 2
					if(ufnorm[0] < 0): ufnorm[0] = 0
				elif(findbound == 3): #triggered on positive y boundary
					prevfindbound = 3
					if(ufnorm[1] > 0): ufnorm[1] = 0
				elif(findbound == 4): #triggered on negative y boundary
					prevfindbound = 4
					if(ufnorm[1] < 0): ufnorm[1] = 0
				print("find: going upflow")
				wp = make_waypoint(auv_location[0]+ufnorm[0], auv_location[1]+ufnorm[1], auv_location[2])
				call_goto(wp, gotoservice, interpolator)


#callback function for particle concentration subscriber
def readconcentration(msg):
	global particle_concentration, auv_location
	particle_concentration = msg.concentration
	auv_location = np.array([msg.position.x, msg.position.y, msg.position.z])


#callback function for auv pose subscriber
def readauvpose(msg):
	global auv_heading
	auv_heading = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])


if __name__=='__main__':
	rospy.init_node('CPT_BBP')

	part_conc_sub = rospy.Subscriber(
		'rexrov2/particle_concentration', 
		ChemicalParticleConcentration, 
		readconcentration)

	auvpos_sub = rospy.Subscriber(
		'rexrov2/pose_gt',
		Odometry,
		readauvpose)

	markerpub = rospy.Publisher('sourcemarker', Marker, queue_size=1)

	interpolator = rospy.get_param('~interpolator', 'lipb')
	
	try:
		rospy.wait_for_service('rexrov2/go_to', timeout=15)
	except rospy.ROSException:
		raise rospy.ROSException('Service not available!')
	
	try:
		goto = rospy.ServiceProxy('rexrov2/go_to', GoTo)
	except rospy.ROSException as e:
		raise rospy.ROSException('Service proxy failed, error=%s', str(e))

	prevstate = -1

	while not rospy.is_shutdown():
		#running the algorithm to quickly makes for some... interesting AUV behavior (namely breakdancing)
		r = rospy.Rate(1)
		r.sleep()

		if(alg_state != prevstate):
			print("Algorithm state: " + s2b[alg_state])
		if(particle_concentration > 0):
			print("Particle concentration = " + str(particle_concentration))
		
		#check algorithm state and run appropriate behavior
		if(alg_state == -1): #initial startup
			alg_state = 0
			prevstate = 0
			wp = make_waypoint(startx, starty, auv_location[2])
			call_goto(wp, goto, interpolator)
		elif(alg_state == 0):	#go to starting pos
			dest = np.array([startx, starty, auv_location[2]])
			if(has_reached(auv_location, dest, R)):
				find_plume(goto, interpolator)
		elif(alg_state == 1):
			prevstate = 1
			find_plume(goto, interpolator)
		elif(alg_state == 2):
			prevstate = 2
			track_in(goto, interpolator)
		elif(alg_state == 3):
			prevstate = 3
			track_out(goto, interpolator)
		elif(alg_state == 4):
			prevstate = 4
			reacquire(goto, interpolator)
		elif(alg_state == 5): #source found
			prevstate = 5
			if(not has_reached(auv_location, lost_pnts[-1], R)): #go to most upflow ldp, which is speculated source
				print("Source: [" + str(lost_pnts[-1][0]) + ", " + str(lost_pnts[-1][1]) + ", " + str(lost_pnts[-1][2]) + "]")
				wp = make_waypoint(lost_pnts[-1][0], lost_pnts[-1][1], lost_pnts[-1][2])
				call_goto(wp, goto, interpolator)

				marker = Marker()
				marker.header.frame_id = 'world'
				marker.header.stamp = rospy.Time.now()
				marker.id = 0
				marker.type = Marker.SPHERE
				marker.action = Marker.ADD
				marker.pose.position.x = lost_pnts[-1][0]
				marker.pose.position.y = lost_pnts[-1][1]
				marker.pose.position.z = lost_pnts[-1][2]
				marker.pose.orientation.x = 0
			  	marker.pose.orientation.y = 0
			  	marker.pose.orientation.z = 0
			  	marker.pose.orientation.w = 1
			  	marker.scale.x = 1.0
			  	marker.scale.y = 1.0
			  	marker.scale.z = 1.0
				marker.color.r = 1.0
				marker.color.g = 1.0
				marker.color.b = 1.0
				marker.color.a = 1.0
				marker.lifetime = rospy.Duration(0)

				markerpub.publish(marker)