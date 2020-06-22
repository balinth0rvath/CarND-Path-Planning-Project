# **Path planning** 

**Path Planning Project**

The goals / steps of this project are the following:
* Generate path that doesn`t break the rules described in the rubric points.
* Ego car must be able to drive for a specified amount of distance without any incident.

## Rubric Points
### Here I will consider the rubic points individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. The code compiles correctly

The code compiles correctly.

---
### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident..
I tested 10 times, the car was able to drive 8-42 miles without incident.

#### 2. The car drives according to the speed limit.
There is a constant value in controller class that limits the max speed. The max speed is 97.5% of speed limit.

#### 3. Max Acceleration and Jerk are not Exceeded.
Points from previous paths and spline used to avoid max acceleration and jerks. Also there is a trajectory smoothness modifier in conroller class to avoid big lateral accelerations.

#### 4. Car does not have collisions.
Ego car follows other cars from safe distance and doesn`t change lanes when it is not safe using a cost function. 

#### 5. The car stays in its lane, except for the time between changing lanes.
Waypoints in frenet coordinates are used to keep car into the lane (see main.cpp #139)

#### 6. The car is able to change lanes
The Controller class (controller.h controller.cpp) is used to select best lane for fast and safe driving. The lane is selected using a cost function.

---
### Reflection

#### Path generation
Spline is used to generate path from anchor points. Anchor points contain previous points, appended to s direction taking account of planned speed and then applied lane changes in d dimension in frenet coordinates. 
The best lane is selected calling Controller::next (See controller.h and controller.cpp) Basically, it selects a best lane and a target speed for the next iteration checking the sensor fusion data and the current speed and current lane. 
There is always a target speed to be accelerated to. This speed can be either the 97.5% of the legal limit or the speed of the vehicle in front of the ego car. 

A cost function is used to select the best lane. (controller.cpp #143) The formula is 

cost[lane] = 1 - exp(-1/front_distance[lane])
where lane is the checked lane, and front_distance is the closest vehichle in front of the ego car in that lane. 

__safety checks__
* If there is a car in the lane of ego vehicle and all the other lanes have 1.0 cost then the speed must be adjusted to keep safe distance (controller.cpp #64)

There is a collision_warning variable for each lane. If it is true, the cost is set to 1 to that lane. (controller.cpp #157)
The variable is set to 1 if:
* There is a car near the ego vehicle in the target lane (controller.cpp #75)
* There is a car approaching in the target lane and has higher speed than the ego vehicle (controller.cpp #84)
* There is a slower car in front of the ego vehicle in the target lane and has lower speed (controller.cpp #92-107)
* If the best lane is the second lane from the current one, then extra attention must be paid to vehicles in the center lane (controller.cpp #106-137)
* No lane change allowed until a specific amount of time when starting from standing position (controller.cpp #160)

Velocity is adjusted adding or subtracting 1 mph in each cycle.




