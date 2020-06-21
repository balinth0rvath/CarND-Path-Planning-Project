#include "controller.h"
#include <iostream>

Controller::Controller()
{
	velocity = 0.0;
	lane = 1;

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

void Controller::setFrontDistances(nlohmann::json j, int prev_size)
{
	double car_s = j[1]["s"];
	front_distances={INF,INF,INF};
	auto sensor_fusion = j[1]["sensor_fusion"];
	for(int i=0; i<sensor_fusion.size(); i++)
	{
		double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
		double check_speed = 2.2 * sqrt(vx*vx+vy*vy);
		double check_car_s = sensor_fusion[i][5];
		//check_car_s += ((double)prev_size*0.02*check_speed); 
		float d = sensor_fusion[i][6];
		int curr_lane =(d>2.0) ? (int)round(((d-2.0)/4.0)) : 0.0;
		//std::cout << "id: " << sensor_fusion[i][0] << "s: " << check_car_s << " lane: " << curr_lane <<  std::endl;	
		double distance = check_car_s - car_s;
		if (d>0.0)
		{
			for(int j=0;j<3;++j)
			{
				if (j==curr_lane && front_distances[j] > distance && distance > 0.0)
				{
					front_distances[j] = distance;
				}
			}	
		}
	}
}

void Controller::setCosts()
{
	for(int i=0;i<3;++i)
	{
		cost[i] = 1 - exp(-1/front_distances[i]);
	}
	std::cout << std::endl;
	auto smallest = std::min_element(std::begin(cost), std::end(cost));
	target_lane = std::distance(std::begin(cost), smallest);
	std::cout << "target: " << lane << std::endl;
}

void Controller::next(nlohmann::json j, int prev_size)
{
	bool too_close=false;
	bool emergency_brake=false;
	bool free=true;
	double target_speed = 48.5;
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

	if(velocity < target_speed)
	{
			velocity += 1.0;
	}
	if (lane < target_lane)
		lane++;

	if (lane > target_lane)
		lane--;
}

