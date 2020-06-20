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
	for(int i=0; i<sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		int car_lane = 0;
		if (d>0.0)
		{
			if (d<2.0)
			{
				car_lane = 0;
			} else
			{
				car_lane = (int)round(((d-2.0)/4.0));
			}

			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = 2.2 * sqrt(vx*vx+vy*vy);
			double check_car_s = sensor_fusion[i][5];
			check_car_s += ((double)prev_size*0.02*check_speed);
			
			if ((car_lane == lane) && (check_car_s > car_s))
			{ 
				double distance  = check_car_s - car_s;
				std::cout << "---distance from it: " << distance << std::endl;

				if (distance < 100.0)
				{
					free = false;
				}

				if (distance < 30.0)
				{
					emergency_brake = true;
				} else if (distance < 60.0)
				{
					too_close = true;
					target_speed = check_speed;
				} 
			}
		}
	}

	if(too_close && (target_speed < velocity))
	{
		std::cout << "too close" << std::endl;
		velocity -= 0.4;
	}

	if (emergency_brake)
	{
			std::cout << "BRAKE" << std::endl;
			velocity -= 2.0;
	}

	else if(velocity < target_speed)
	{
		std::cout << "approach" << std::endl;
		if (free)
		{
			velocity += 1.2;
		} else
		{
			velocity += 0.2;
		}
	}

}

