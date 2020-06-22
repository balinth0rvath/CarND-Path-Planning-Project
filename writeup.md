# **Path planning** 

**Path Planning Project**

The goals / steps of this project are the following:
* Generate path that doesnt break the rules described in the rubric points 
* Ego car must be able to drive for a specified amount of miles without incident

## Rubric Points
### Here I will consider the rubic points individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. The code compiles correctly

The code compiles correnctly

---
### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident..
I tested 10 times, the car was able to drive at least 8 miles without incident

#### 2. The car drives according to the speed limit.
There is a constant value in controller class that limits the max speed. The max speed is 97.5% of speed limit

#### 3. Max Acceleration and Jerk are not Exceeded.
Points from previous paths and spline used to avoid max acceleration and jerks. Also there is a trajectory smoothness modifier in conroller class to avoid big lateral accelerations

#### 4. Car does not have collisions.
Ego car follows other cars from safe distance and dont change lane when it is  

#### 5. The car stays in its lane, except for the time between changing lanes.

#### 6. The car is able to change lanes

---
### Reflection

#### 1. There is a reflection on how to generate paths.


