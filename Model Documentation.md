# CarND-Path-Planning-Project Model Documentation
Self-Driving Car Engineer Nanodegree Program -- Path Planning Project
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Results
The car is able to drive at least 4.32 miles without incident.  
It costed about 7 mins to complete 5 miles drive without speed limit violation.   
During the manouvering, the car is able to do lane change accordingly.    
No big acceleration and jerk during the whole trip.   


### Solutions

#### Overview
The path planing project was based on the proposals in project workthrough part.
It inclues following main parts:
1. Prediction -- To understand the enviornment and nearby cars.
2. Behaviour Decision -- To make a decision which manouver (direction, speed change ..) should the car take.
3. Trajectory Generation -- To generate target trajectory of the car

#### Prediction
Line: 249-316
In prediction part, the code utilize the sensor fusion data to detect nearby cars around the ego car.
The ahead car means there is a car ahead of ego car and it's located in critical space (e.g.30m). It is detected based on the lane it located and future position at the end of last planed path. 
The right/left car is detected if the car is in different lane and the future position in critical space.

The critical space range for left car detection is smaller than right ar detection. In this way, left lane change is more sensitive.  

#### Behaviour Decision
Line: 318-352
In this part, ego car's behaviour is planned according to the information (left, ahead, right car) from prediction part.

NO car around the ego car -- speed increase, middle lane
Ahead Car && !Left car -- no speed change, change to lane left (higer priority)
Ahead Car && !Right car -- no speed change, change to lane right
Ahead Car && Right car && Left car--  speed decrease, no lane change

#### Trajectory Generation
Line 354-469
In this part, the trajectory is generated based on the target lane and speed change command from behaviour decision part.

According to the suggestion in workthrough part, spline is used to generate trajectory.
Last two points of previous trajectory and future 3 target points (30m, 60m, 90m in S direction) is used for spline calcultion. 

The previous path is used for the new trajectory generation. The points are copied to the new trajctory. The rest points (50 - pre_size) are calulated based on the target speed and certain distance ahead (e.g.30m).



