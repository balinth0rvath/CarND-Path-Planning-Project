#include "json.hpp"

class Controller {
public:
	Controller();
	~Controller();

	void next(nlohmann::json obj,int prev_size);
	double getVelocity();
	double getTrajectoryModifier();
	int getLane();
private:
	long tick;
	long prev_tick;
	const double speed_limit = 50;
	double INF = 99999.9;
	void setFrontDistances(nlohmann::json j, int prev_size);
	void setCosts();
	void setTargetSpeed();
	double velocity;
	int lane;
	int prev_lane;
	double target_speed;
	std::vector<double> front_distances;
	std::vector<bool> collision_warnings;
	
	std::vector<double> cost;
};
