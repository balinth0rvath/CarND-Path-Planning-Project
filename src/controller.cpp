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
	for(int i=0; i<sensor_fusion.size(); i++)
	{
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = 2.2 * sqrt(vx*vx+vy*vy);
		double check_car_s = sensor_fusion[i][5];
		//check_car_s += ((double)prev_size*0.02*check_speed); 
		float d = sensor_fusion[i][6];
		int check_car_lane =(d>2.0) ? (int)round(((d-2.0)/4.0)) : 0.0;
		//std::cout << "id: " << sensor_fusion[i][0] << "s: " << check_car_s << " lane: " << curr_lane <<  std::endl;	
		double distance = check_car_s - car_s;
		if (d>0.0)
		{
			for(int j=0;j<3;++j)
			{
				if (j==check_car_lane && front_distances[j] > distance && distance > 0.0)
				{
					front_distances[j] = distance;

					if (check_car_lane == lane && distance < 27.0)
						target_speed = check_speed;
				}
				if (j==check_car_lane && j!=lane && distance < 5.0 && distance > -8.0)
				{
					//std::cout << "collision " << j << "  distance " << distance <<  std::endl;
					collision_warnings[j] = true;
				}
				double speed_delta = check_speed - velocity;
				if (j==check_car_lane && j!=lane && distance < 0.0 && distance > speed_delta * -4.0 )
				{
					//std::cout << "incoming " << j <<" distance " << distance << std::endl;
					collision_warnings[j] = true;
				}

				if (j==check_car_lane && j!=lane && distance > 0.0 && distance < 30.0 && speed_delta < -15.0)
				{
					collision_warnings[j] = true;
					//std::cout << "PASSING FAST" << j <<" distance " << distance << std::endl;
				} else if (j==check_car_lane && j!=lane && distance > 0.0 && distance < 10.0 && speed_delta < -1.0)
				{
					collision_warnings[j] = true;
					//std::cout << "PASSING slow close" << j <<" distance " << distance << std::endl;
				}
				bool avoid_crossing = false;
				if (lane!=check_car_lane && check_car_lane == 1)
				{ 
					if (distance < 19.0 && distance > -6.0 && fabs(speed_delta) > 8.0)	
					{
						avoid_crossing = true;
						std::cout << "AVOID speed CROSSING" << std::endl;
					}
					if (distance < 10.0 && distance > -5.0 && fabs(speed_delta) <= 8.0)
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
	for(int i=0;i<3;++i)
	{
		cost[i] = 1.0 - exp(-1/front_distances[i]);
		if (collision_warnings[i])
			cost[i] = 1.0;

		if (tick<50 && i!=lane)
		{
			std::cout << "avoid" << tick << std::endl;
			cost[i] = 1.0;
		}	
	}

	for(int i=0;i<3;++i)
	{
		std::cout << cost[i] << " ";
	}
	std::cout << std::endl;
	auto smallest = std::min_element(std::begin(cost), std::end(cost));
	lane = std::distance(std::begin(cost), smallest);

}


void Controller::next(nlohmann::json j, int prev_size)
{
	tick++;
	bool too_close=false;
	bool emergency_brake=false;
	bool free=true;
	target_speed = speed_limit * 0.975;
	// Main car's localization Data
	double car_x = j[1]["x"];
	double car_y = j[1]["y"];
	double car_s = j[1]["s"];
	double car_d = j[1]["d"];
	double car_yaw = j[1]["yaw"];
	double car_speed = j[1]["speed"];

	// Previous path data given to the Planner
	auto previous_path_x = j[1]["previous_path_x"];
	auto previous_path_y = j[1]["previous_path_y"];
	// Previous path's end s and d values 
	double end_path_s = j[1]["end_path_s"];
	double end_path_d = j[1]["end_path_d"];

	// Sensor Fusion Data, a list of all other cars on the same side 
	//   of the road.
	auto sensor_fusion = j[1]["sensor_fusion"];

	cost = {0.0,0.0,0.0};
	setFrontDistances(j,prev_size);
	setCosts();

	if(velocity < target_speed - 1.0)
	{
			velocity += 1.0;
	}

	if(velocity > target_speed + 1.0)
	{
			velocity -= 1.0;
	}

}

