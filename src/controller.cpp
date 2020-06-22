#include "controller.h"
#include <iostream>

Controller::Controller()
{
	velocity = 0.0;
	lane = 1;
	tick = 0;

}

Controller::~Controller()
{
}

double Controller::getVelocity()
{
	return velocity;
}

int Controller::getLane()
{
	return lane;
}

double Controller::getTrajectoryModifier() 
{
	// Set smoother path for high speed
	double modifier = 2.1 * velocity / speed_limit;
	if (modifier < 1.0)
		modifier = 1.0;
	return modifier; 
}
void Controller::setFrontDistances(nlohmann::json j, int prev_size)
{
	double car_s = j[1]["s"];
	front_distances={INF,INF,INF};
	collision_warnings={false,false,false};
	auto sensor_fusion = j[1]["sensor_fusion"];

	// this loop does two things:
	// 1.  calculates the nearest cars in front of the ego car in each lane
	// 2.	 calculates warnings used in cost function

  // iterate over all cars in sensor range
	for(int i=0; i<sensor_fusion.size(); i++)
	{
    // calcuate speed s and d
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = 2.2 * sqrt(vx*vx+vy*vy);
		double check_car_s = sensor_fusion[i][5];
		float d = sensor_fusion[i][6];
		int check_car_lane =(d>2.0) ? (int)round(((d-2.0)/4.0)) : 0.0;
		double distance = check_car_s - car_s;
		// skip opposite lanes
		if (d>0.0)
		{
			// j is the target lane for calculating cost 
			for(int j=0;j<3;++j)
			{
				if (j==check_car_lane && front_distances[j] > distance && distance > 0.0)
				{
					// If the checked is closer than the stored one and it is in front of the ego car,  then replace that with this one
					front_distances[j] = distance;

					// If the checked car is in our lane and it is too close then match speed
					if (check_car_lane == lane && distance < 29.0)
						target_speed = check_speed;
				}
				// **
				// * SAFETY CHECKS. Cost function later forbids with 1.0 cost any of events calculated here
				// **
			
				// If the checked car is not in our lane and it is too close to the ego car
        // then set the cost of that lane to the upper limit
				if (j==check_car_lane && j!=lane && distance < 10.0 && distance > -11.0)
				{
					std::cout << "collision " << j << "  distance " << distance <<  std::endl;
					collision_warnings[j] = true;
				}
				double speed_delta = check_speed - velocity;

				// If the checked car is not in our lane and it is behind us and approaches with
				// much higher speed the avoid that lane
				if (j==check_car_lane && j!=lane && distance < 0.0 && distance > speed_delta * -4.0 )
				{
					std::cout << "incoming " << j <<" distance " << distance << std::endl;
					collision_warnings[j] = true;
				}

				// If the checked car is not in our lane and it is much slower then avoid that lane
				// even it is far from ego vehicle
				if (j==check_car_lane && j!=lane && distance > 0.0 && distance < 31.0 && speed_delta < -14.0)
				{
					collision_warnings[j] = true;
					std::cout << "PASSING FAST" << j <<" distance " << distance << std::endl;

					// If the checked car is not in our lane and it is a bit slower then avoid than lane
					// from a shorter distance
				} else if (j==check_car_lane && j!=lane && distance > 0.0 && distance < 15.0 && speed_delta < -1.0)
				{
					collision_warnings[j] = true;
					std::cout << "PASSING slow close" << j <<" distance " << distance << std::endl;
				}
				bool avoid_crossing = false;

	
				// Changing two lanes at once is dangerous. This logic prohibits double lane crossing
				// before/behind a  car in the middle lane depending on speed delta
				if (lane!=check_car_lane && check_car_lane == 1)
				{ 
					if (distance < 28.0 && distance > -11.0 && fabs(speed_delta) > 8.0)	
					{
						avoid_crossing = true;
						std::cout << "AVOID speed CROSSING" << std::endl;
					}
					if (distance < 24.0 && distance > -10.0 && fabs(speed_delta) > 4.0 && fabs(speed_delta) <=8.0)	
					{
						avoid_crossing = true;
						std::cout << "AVOID speed CROSSING" << std::endl;
					}
					if (distance < 20.0 && distance > -9.0 && fabs(speed_delta) <= 4.0)
					{
						avoid_crossing = true;
						std::cout << "AVOID slow crossing" << std::endl;	
					}
				}
				if (avoid_crossing)
				{
					if (lane==0)
							collision_warnings[2]= true;

					if (lane==2)
						collision_warnings[0]= true;

				}	
			}	
		}
	}
}

void Controller::setCosts()
{
	// calculate cost for each lane using exponential function
  // longer distances for each lanes are rewarded. Cost is 
  // skipped at 0.01 to avoid unnecessary lane changes generated
  // of 200m+ traffic changes
	for(int i=0;i<3;++i)
	{
		cost[i] = 1.0 - exp(-1/front_distances[i]);

		if (cost[i]<0.01)
			cost[i]=0.01;

		// forbid all changes that are dangerous according to safety checks up there
		if (collision_warnings[i])
			cost[i] = 1.0;

		// For a short time, prohibit all lane changes on a highway when starting from standing position
		// regardless of sensor data	
		if (tick<50 && i!=lane)
		{
			cost[i] = 1.0;
		}	
	}

	// Get the best cost lane
	auto smallest = std::min_element(std::begin(cost), std::end(cost));
	double best_lane = std::distance(std::begin(cost), smallest);

	// If the best one is the same to the current one then dont change lane. It could cause
	// unnecessray changes leading to break the 3 sec out-of-lane rule.	
	if (cost[best_lane]<cost[lane])
		lane = best_lane;

}


void Controller::next(nlohmann::json j, int prev_size)
{
	tick++;
	std::cout << std::endl;
	// set speed limit close to the legal limit
	target_speed = speed_limit * 0.975;

	// Sensor Fusion Data, a list of all other cars on the same side 
	//   of the road.
	auto sensor_fusion = j[1]["sensor_fusion"];

	cost = {0.0,0.0,0.0};

	// set front distances of cars in the same direction. Set collision warnings
	setFrontDistances(j,prev_size);

	// call the cost function. Set the target lane and the target speed
	setCosts();

	// modify velocity towards the target speed taking account of the max acceleration
	if(velocity < target_speed - 0.3)
	{
			velocity += 0.5;
	}

	if(velocity > target_speed + 0.3)
	{
			velocity -= 0.5;
	}
}

