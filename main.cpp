#include <iostream>
#include <string>
#include <conio.h>
#include "Robot_model.h"
#include "vrep_bridge.h"


using namespace std;


int main()
{
	Robot_model robot_;
	VRepBridge vrep;
	bool flag = true;
	VectorXd qd(6); //set the size by 6
	qd.setZero();
	VectorXd qdot(6);
	qdot.setZero();
	Matrix3d baseOrientation;
	baseOrientation << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	VectorXd Answer(6);
	robot_.updateTransformMatrix();
	robot_.updateRobotmodel();
	robot_.UpdateState(qd, qdot);

	cout << "Insert Key" <<endl;

	while (vrep.simConnectionCheck() && flag) //Vrep is connected at simConnectionCheck()
	{

		vrep.read();

		if (_kbhit())
		{
			int key = _getch();
			switch (key)
			{
			case 'j':
				cout << "JacobianBasedSolver" << endl;
				Answer = robot_.JacobianBasedSolver(vrep.target_x_, baseOrientation);
				break;
			case 'a':
				cout << "Analytic IK" << endl;
				Answer = robot_.analyticIKSolver(vrep.target_x_, baseOrientation);
				break;
			case 'q':
				cout << "finished" << endl;
				flag = false;
				break;
			case 's':
				cout << "Simulated" << endl;
				vrep.set_desired_q_(Answer);
				vrep.write();
				vrep.simLoop();
				break;
			default:
				break;
			}
		}
	}
	return 0;
}