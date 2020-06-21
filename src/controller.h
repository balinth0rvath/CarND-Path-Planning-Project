#include "json.hpp"
enum class State {
	kFollow,
	kChangeLeft,
	kChangeRight,
	kPrepareToChangeLeft,
	kPrepareToChangeRight
};

class Controller {
public:
	Controller();
	~Controller();

	void next(nlohmann::json obj,int prev_size);
	double getVelocity();
	int getLane();
private:
	double INF = 99999.9;
	void setFrontDistances(nlohmann::json j, int prev_size);
	void setCosts();
	double velocity;
	int lane;
	int state;
	int target_lane;
	std::vector<double> front_distances;
	
	std::vector<double> cost;
};
