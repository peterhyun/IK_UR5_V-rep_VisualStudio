#include "vrep_bridge.h"

void VRepBridge::simxErrorCheck(simxInt error)
{
	string errorMsg;
	switch (error)
	{
	case simx_error_noerror:
		return;	// no error
		break;
	case simx_error_timeout_flag:
		errorMsg = "The function timed out (probably the network is down or too slow)";
		break;
	case simx_error_illegal_opmode_flag:
		errorMsg = "The specified operation mode is not supported for the given function";
		break;
	case simx_error_remote_error_flag:
		errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
		break;
	case simx_error_split_progress_flag:
		errorMsg = "The communication thread is still processing previous split command of the same type";
		break;
	case simx_error_local_error_flag:
		errorMsg = "The function caused an error on the client side";
		break;
	case simx_error_initialize_error_flag:
		errorMsg = "simxStart was not yet called";
		break;
	default:
		errorMsg = "Unknown error.";
		break;
	}

	cout << "[ERROR] An error is occured. code = " << error << endl;
	cout << " - Description" << endl;
	cout << " | " << errorMsg << endl;

	throw std::string(errorMsg);
}

void VRepBridge::dataInit()
{
	current_q_.resize(MOTORNUM);
	current_q_.setZero();
	target_x_.resize(3);
	target_x_.setZero();
}

void VRepBridge::simInit()
{
	simxFinish(-1);
	clientID_ = simxStart("127.0.0.1", 19997, true, true, 2000, 5);
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}


	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	cout << "[INFO] V-Rep connection is established." << endl;

}

void VRepBridge::write()
{
	simxSetJointPosition(clientID_, motorHandle_[0], current_q_(0), simx_opmode_streaming);
	simxSetJointPosition(clientID_, motorHandle_[1], current_q_(1), simx_opmode_streaming);
	simxSetJointPosition(clientID_, motorHandle_[2], current_q_(2), simx_opmode_streaming);
	simxSetJointPosition(clientID_, motorHandle_[3], current_q_(3), simx_opmode_streaming);
	simxSetJointPosition(clientID_, motorHandle_[4], current_q_(4), simx_opmode_streaming);
	simxSetJointPosition(clientID_, motorHandle_[5], current_q_(5), simx_opmode_streaming);
}

void VRepBridge::read()
{
	for (size_t i = 0; i < MOTORNUM; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		current_q_(i) = data;

	}

	simxFloat data_[3];
	simxGetObjectPosition(clientID_, Target, -1, data_, simx_opmode_streaming);

	//cout << data_[0]<<" " << data_[1]<<" " << data_[2] << endl;
	for (int i = 0; i < 3; i++)
	{
		target_x_(i) = data_[i];
	}
}


void VRepBridge::getHandle()
{
	cout << "[INFO] Getting handles." << endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "UR5_joint1", &motorHandle_[0], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "UR5_joint2", &motorHandle_[1], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "UR5_joint3", &motorHandle_[2], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "UR5_joint4", &motorHandle_[3], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "UR5_joint5", &motorHandle_[4], simx_opmode_oneshot_wait));
	simxErrorCheck(simxGetObjectHandle(clientID_, "UR5_joint6", &motorHandle_[5], simx_opmode_oneshot_wait));

	simxErrorCheck(simxGetObjectHandle(clientID_, "Sphere", &Target, simx_opmode_oneshot_wait));

	cout << "[INFO] The handle has been imported." << endl;
}