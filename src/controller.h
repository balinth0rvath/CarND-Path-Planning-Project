#include "json.hpp"

class Controller {
public:
	Controller();
	~Controller();

	/**
	 * next helps calculate the next trajectory passing back
	 * a best lane and a speed
	 * @param json object containing localization data 
   * map is not needed since it uses frenet coordinates
   * @param prev size contains the previous path size
	 * used to calculate the actual position(not delayed)
	 */
	void next(nlohmann::json obj,int prev_size);
	/**
	 * getVelocity returns the calculated speed
	 */
	double getVelocity();
	/**
	 * getTrajectoryModifier returns a rate used to smooth the path 
	 */
	double getTrajectoryModifier();

	/**
	 * getLane returns the best lane 
	 */
	int getLane();
private:
  // Number of calculations since the start
	long tick;

	// Spped limit to obey	
	const double speed_limit = 50;

	// Infinity distance used in other car distance calculation
	double INF = 99999.9;

	// Calculates the closest cars in the right lanes. Cars backwards are not counted here
	void setFrontDistances(nlohmann::json j, int prev_size);

	// Calculates cost values for each lane
	void setCosts();

	// Calcuated speed
	double velocity;

	// Calculated best lane
	int lane;

	// the target of the actual velocity
	double target_speed;

	// closest cars in each right lane in front of ego car
	std::vector<double> front_distances;

	// warnings for each lane used in cost functions. If a lane has true value
	// here the cost of changing to that lane is 1.0 
	std::vector<bool> collision_warnings;
	
	// Cost values for each lane 
	std::vector<double> cost;
};
