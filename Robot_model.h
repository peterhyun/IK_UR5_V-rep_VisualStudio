#ifndef __ROBOT_STATE__
#define __ROBOT_STATE__

#include <Eigen/Dense>
#include "rbdl/rbdl.h"
using namespace RigidBodyDynamics;
using namespace Eigen;
const int dof_ = 6;

class Robot_model
{
private:

	Model * model;
	unsigned int body_id[dof_];
	Body body[dof_];
	Joint joint[dof_];
	Vector3d joint_pos[dof_];

	Math::Vector3d com_pos[dof_];
	Math::VectorNd Q;
	Math::VectorNd QDot;
	Math::VectorNd Tau;
	Math::VectorNd QDDot;
public:

	void UpdateState(const VectorXd q, const VectorXd qdot)
	{
		Q = q; //각 관절의 각도들! matlab으로 치면 substitute으로 침!
		QDot = qdot;
		QDDot.setZero();
	};
	void updateRobotmodel();
	Vector3d getPosition();
	Vector3d getPosition2(VectorXd q);
	Matrix3d getOrientation();
	Matrix3d getOrientation2(VectorXd q);
	MatrixXd getJacobian();
	MatrixXd getJacobian2(VectorXd q);
	Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd);
	VectorXd solveIK1();
	VectorXd analyticIKSolver(Vector3d x_d, Matrix3d ori_d);
	VectorXd JacobianBasedSolver(Vector3d x_d, Matrix3d ori_d);
	void updateTransformMatrix();
};
#endif