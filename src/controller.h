#include "json.hpp"
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

};
