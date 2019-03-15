# Plume Tracing for UUV Simulator
[**Demo Video**](https://www.youtube.com/watch?v=SYhh60iHcdc)  
  
![pt logo](https://github.com/ethanjpark/ethanjpark.github.io/blob/master/links/uuvsim.png)  
  
This is my MSR winter quarter project. I will be using the plume simulation feature in the [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) to implement various plume detection algorithms and methods. UUV Simulator was developed in the scope of the EU ECSEL project 662107 SWARMS for unmanned underwater vehicle simulation based on Gazebo and ROS by [Musa Morena Marcusso Manhaes](https://github.com/musamarcusso). Since it began development in 2016 it seems to have gained significant traction in the maritime ROS and robotics community, especially in Europe. It was a topic of discussion at underwater ROS workshops such as [BTS 2018](https://discourse.ros.org/t/bts-2018-workshop-adoption-of-conventions-in-the-underwater-ros-community/5389) and a underwater ROS workshop [at Woods Hole Oceanographic Institution](https://discourse.ros.org/t/underwater-ros-workshop-woods-hole-oceanographic-institution/5197).

With this project I aim to familiarize myself with a potentially useful tool for future work and study in the field of underwater robots. Furthermore, I hope to continue to learn about the unique challenges that face robots in underwater environments through implementation of plume detection, a very common application of underwater robots in both commercial and scientific uses. 

## Running the plume tracing  
First, run:  
`roslaunch pt_uuvsim start_pt_environ.launch` 
  
Once the RViz window loads, run:  
`roslaunch pt_uuvsim start_plume.launch`  
  
*Note: This two-part launching procedure is due to the scripts in start_plume.launch failing to execute if called on too quickly after the hydrodynamic plume server creation script has been called in the start_pt_environ.launch. Waiting for the RViz window to load, therefore, is merely to allow enough time for the plume server to be created*  

## Potential Issues
First, though a major preliminary concern was the simulator's compatibility with ROS Melodic, fortunately this proved to be a non-issue and the simulator works flawlessly on Melodic - assuming that all the required packages and APIs are present on the machine and ROS environment.  
  
The latter is where I encountered problems, as I was missing [message_to_tf](https://wiki.ros.org/message_to_tf), [geographic_msgs](https://wiki.ros.org/geographic_msgs), and [GeographicLib](https://geographiclib.sourceforge.io), and these dependencies were not handled by installation or in the documentation. However, judging by how commonly these packages seem to be depended on by other ROS projects, assuming that a user for this simulator would likely already have these packages is fairly reasonable.  
  
As pointed out by multiple users in the Issues section of the UUV Simulator repository, there are a few mismatched filepaths and names in the quickstart launch files that are referred to in the documentation and the tutorials. *(i.e. executing a launch file in uuv_descriptions, when it is actually in uuv_gazebo_worlds)* The author has not yet fixed these, so addressing them are left to the user.

## Code Runthrough - Simulator Shenanigans
In the process of working through the tutorials and tweaking the relevant bits for use in my final implementation, I have copied over various scripts, configuration/Rviz files, and launch files from the UUV Simulator package to minimize modifications to the simulator package. Their original counterparts are noted in comments in their respective files.  
  
To mention the bits that are relevant to the final, plume tracing, implementation; I used the default PID positioning controller ([launch file](https://github.com/ethanjpark/Plume_Tracing_UUV_Sim/blob/master/launch/tutorial_dp_controller.launch)) from the tutorials to handle the movement of the robot once it was given a destination, the [set_currvel](https://github.com/ethanjpark/Plume_Tracing_UUV_Sim/blob/master/scripts/set_currvel) script to set the direction and magnitude of the current flow, and the [turbulent_plume](https://github.com/ethanjpark/Plume_Tracing_UUV_Sim/blob/master/scripts/turbulent_plume) script to create the plume. The latter is also where the user can set the plume source location and boundaries, among other parameters.

## Code Runthrough - Plume Tracing Code
The implementation of the [chemical plume tracing paper](https://ieeexplore.ieee.org/document/1522521) is done in [cptbbp.py](https://github.com/ethanjpark/Plume_Tracing_UUV_Sim/blob/master/src/cptbbp.py). The code overall keeps track of the algorithm state, which is akin to which behavior the algorithm is currently performing. I also use the Waypoint message and GoTo service call - both from uuv_control_msgs, a package in the UUV Simulator - to create a message containing the desired destination, and tell the AUV to move to said destination, respectively.  
  
**Waypoint message creation function, Line 81:** `def make_waypoint(newx,newy,newz)`  
**GoTo service call, Line 401:** `goto = rospy.ServiceProxy('rexrov2/go_to', GoTo)`  
  
The chemical sensor readings are obtained via subscribing to the *rexrov2/particle_concentration* topic and the AUV position via *rexrov2/pose_gt*.  
  
**Sensor data subscriber, Line 381:** `part_conc_sub = rospy.Subscriber('rexrov2/particle_concentration', ChemicalParticleConcentration, readconcentration)`  
**AUV position subscriber, Line 386:** `auvpos_sub = rospy.Subscriber('rexrov2/pose_gt', Odometry, readauvpose)`  
  
Lastly, a marker (white sphere) is also published once the source has been found to provide a visual cue for where the AUV thinks the source is.  
  
**Source marker publisher, Line 391:** `markerpub = rospy.Publisher('sourcemarker', Marker, queue_size=1)`  

### Behaviors explained
Before any further details, it is important to remember that the robot is aware of the direction of current flow, and therefore also the direction of upflow. In a real robot this would be done via sensors to detect the current velocity; in my code I have simply hardcoded a constant numpy vector to be used in calculations.  
  
**GoTo** - initial behavior, the robot simply moves to the starting location. *(Line 418-422)*  
  
**Find** - the robot traverses the operational area to find the plume, going across the direction of flow. Basically large-scale zig-zag. *(Line 319)*  
  
**Track-In** - the robot has found the plume, and proceeds to travel up the plume (upflow). Based on the robot's current heading when plume is detected, the robot determines which side of the plume it would exit out of, and travels in a diagonal manner to the direction of flow. This diagonal heading is a constant angle offset (side depends on the exit side determined just before) from the direction of upflow. Robot stays in Track-In as long as it detects the plume and stores its location as a last detected point, otherwise it will add the last detected point to a separate list and go to Track-Out. *(Line 129)*  
  
**Track-Out** - the first resort for when the robot has lost the plume. Uses the last detected point as a backtracking point. If it reaches this last detected point or it finds the plume again, it checks if it can declare a source. If there are more than three last detected points in the stored list *and* the distance between these three points in the direction of upflow is less than 4 meters, then source is declared to be the most recent point in the list and algorithm finishes. If source cannot be declared, then behavior goes to Track-In (if plume was found) or Reacquire (if robot reached last detected point). *(Line 193)*  
  
**Reacquire** - the second resort for when the robot has lost the plume. Here, the robot performs a special bowtie-shaped maneuver centered around each of the last detected points stored in the list. Starting from the most recent last detected point, this maneuver is performed until either the plume is found, or there are no more last detected points to perform the maneuver around. For the former, the behavior switches to Track-In. For the latter, the behavior switches to Find. *(Line 257)*  
  
Also of interest, in the paper there is a last behavior that triggers after the source has been declared, called "Post-Declare Maneuvers". In a nutshell, this behavior gathers data from the proximity around the declared source - i.e. sonar mappings of the seabed - for later human processing and analysis to confirm this declared source.  
  
3.15.2019 - Ethan Park (park.ethan@u.northwestern.edu)
