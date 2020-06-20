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
	double velocity;
	int lane;
	int state;

	std::vector<double> cost;
};
