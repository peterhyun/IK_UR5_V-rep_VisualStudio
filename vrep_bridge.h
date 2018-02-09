#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>

extern "C" {
#include "remoteApi/extApi.h"
}

using namespace std;

const int MOTORNUM = 6;	/// < Depends on simulation envrionment

class VRepBridge
{
private:
	typedef std::function<void()> callfunc; // loop callback function

public:
	VRepBridge() : tick_(0)
	{
		dataInit();
		simInit();
		getHandle();
	}
	~VRepBridge()
	{
		simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
		simxFinish(clientID_);
	}

	bool simConnectionCheck()
	{
		return (simxGetConnectionId(clientID_) != -1);
	}
	void simLoop()
	{
		//loopCallbackFunc();
		tick_++;
		simxSynchronousTrigger(clientID_);
	}
	void write();
	void read();


public:
	Eigen::VectorXd current_q_;
	Eigen::VectorXd target_x_;

	const size_t getTick() { return tick_; }

private:
	simxInt clientID_;
	simxInt motorHandle_[MOTORNUM];	/// < Depends on simulation envrionment
	simxInt objectHandle_;
	simxInt Target;

	size_t tick_;
	//callfunc loopCallbackFunc;

	void simxErrorCheck(simxInt error);
	void dataInit();
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment

public:
	void set_desired_q_(Eigen::VectorXd Answer) {
		current_q_ = Answer;
	}
};
