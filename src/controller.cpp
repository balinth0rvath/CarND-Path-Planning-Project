#include "controller.h"

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

	for(int i=0; i<sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		if(d<(2+4*lane+2) && d>(2+4*lane-2))
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt(vx*vx+vy*vy);
			double check_car_s = sensor_fusion[i][5];

			check_car_s += ((double)prev_size*0.02*check_speed);
			if ((check_car_s > car_s) && ((check_car_s-car_s) < 40))
			{
				too_close = true;
				if (lane > 0)
				{
					lane = 0;
				}
			}
		}	
	}

	if(too_close)
	{
		velocity -= 1.;
	}
	else if(velocity < 48.8)
	{
		velocity += 1.;
	}

}

