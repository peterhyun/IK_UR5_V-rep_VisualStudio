#include "Robot_model.h"
#include "TransformationM.h"

#define d1 0.0746
#define a2 0.4251
#define a3 0.3922
#define d4 0.1100
#define d5 0.0948
#define d6 -0.0144
/*
T01 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] * [cos(q1) -sin(q1) 0 0;sin(q1) cos(q1) 0 0;0 0 1 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 d1;0 0 0 1];
T12 = [1 0 0 0; 0 0 -1 0; 0 1 0 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] * [cos(q2) -sin(q2) 0 0;sin(q2) cos(q2) 0 0;0 0 1 0;0 0 0 1] * [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T23 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 a2;0 1 0 0;0 0 1 0;0 0 0 1] * [cos(q3) -sin(q3) 0 0;sin(q3) cos(q3) 0 0;0 0 1 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
T34 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1] * [1 0 0 a3;0 1 0 0;0 0 1 0;0 0 0 1] * [cos(q4) -sin(q4) 0 0;sin(q4) cos(q4) 0 0;0 0 1 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 d4;0 0 0 1];
T45 = [1 0 0 0; 0 0 -1 0;0 1 0 0; 0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] * [cos(q5) -sin(q5) 0 0;sin(q5) cos(q5) 0 0;0 0 1 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 d5;0 0 0 1];
T56 = [1 0 0 0; 0 0 1 0; 0 -1 0 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1] * [cos(q6) -sin(q6) 0 0;sin(q6) cos(q6) 0 0;0 0 1 0;0 0 0 1] * [1 0 0 0;0 1 0 0;0 0 1 d6;0 0 0 1];
*/
using namespace std;
void Robot_model::updateTransformMatrix() {
	eye << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	temp1 << 1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 0,
		0, 0, 0, 1;
	temp2 << 1, 0, 0, 0,
		0, 0, 1, 0,
		0, -1, 0, 0,
		0, 0, 0, 1;
	temp3 << 1, 0, 0,
		0, 0, -1,
		0, 1, 0;
	A2 << 1, 0, 0, a2,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	A3 << 1, 0, 0, a3,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	D1 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, d1,
		0, 0, 0, 1;
	D4 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, d4,
		0, 0, 0, 1;
	D5 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, d5,
		0, 0, 0, 1;
	D6 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, d6,
		0, 0, 0, 1;
}
void Robot_model::updateRobotmodel()
{
	model = new Model();
	model->gravity = Vector3d(0., 0., -9.81); //not that important to me right now. Just the gravity factor

	double mass[dof_];
	mass[0] = 2.7;
	mass[1] = 2.7;
	mass[2] = 2.7;
	mass[3] = 2.7;
	mass[4] = 1.7;
	mass[5] = 1.6;

	//Axis seen from global coordinate
	Vector3d axis[dof_];
	axis[0] = Vector3d(0.0, 0.0, 1.0);
	axis[1] = Vector3d(0, -1, 0);
	axis[2] = Vector3d(0, -1, 0);
	axis[3] = Vector3d(0, -1, 0);
	axis[4] = Vector3d(0, 0, -1);
	axis[5] = Vector3d(0, -1, 0);

	Math::Vector3d inertia[dof_];
	inertia[0] = Vector3d(4.000e-02, 4.000e-02, 4.000e-02);
	inertia[1] = Vector3d(4.000e-02, 4.000e-02, 4.000e-02);
	inertia[2] = Vector3d(4.000e-02, 4.000e-02, 4.000e-02);
	inertia[3] = Vector3d(4.000e-02, 4.000e-02, 4.000e-02);
	inertia[4] = Vector3d(4.000e-02, 4.000e-02, 4.000e-02);
	inertia[5] = Vector3d(4.000e-02, 4.000e-02, 4.000e-02);

	//frame i+1 seen from frame i == joint_pos[i]
	joint_pos[0] = Vector3d(0.0000, 0.0000, 0.0085);
	joint_pos[1] = Vector3d(0.0000, -0.0703, 0.0746) - joint_pos[0];
	joint_pos[2] = Vector3d(0.4251, -0.0703, 0.0746) - joint_pos[1] - joint_pos[0];
	joint_pos[3] = Vector3d(0.8173, -0.0703, 0.0746) - joint_pos[2] - joint_pos[1] - joint_pos[0];
	joint_pos[4] = Vector3d(0.8173, -0.1100, 0.0290) - joint_pos[3] - joint_pos[2] - joint_pos[1] - joint_pos[0];
	joint_pos[5] = Vector3d(0.8173, -0.0956, -0.0202) - joint_pos[4] - joint_pos[3] - joint_pos[2] - joint_pos[1] - joint_pos[0];

	//center of mass
	com_pos[0] = Vector3d(-0.017, -0.0, 0.2447) - joint_pos[0];
	com_pos[1] = Vector3d(0.0189, 0.0, 0.3847) - joint_pos[1] - joint_pos[0];
	com_pos[2] = Vector3d(0.0189, 0.0, 0.6447) - joint_pos[2] - joint_pos[1] - joint_pos[0];
	com_pos[3] = Vector3d(-0.0171, -0.0, 0.7848) - joint_pos[3] - joint_pos[2] - joint_pos[1] - joint_pos[0];
	com_pos[4] = Vector3d(-0.0171, -0.0, 1.0397) - joint_pos[4] - joint_pos[3] - joint_pos[2] - joint_pos[1] - joint_pos[0];
	com_pos[5] = Vector3d(0.8173, -0.0956, -0.0202) - joint_pos[5] - joint_pos[4] - joint_pos[3] - joint_pos[2] - joint_pos[1] - joint_pos[0];

	for (int i = 0; i < dof_; i++) {
		body[i] = Body(mass[i], com_pos[i], inertia[i]);
		joint[i] = Joint(JointTypeRevolute, axis[i]);
		if (i == 0)
			body_id[i] = model->AddBody(0, Math::Xtrans(joint_pos[i]), joint[i], body[i]);
		else
			body_id[i] = model->AddBody(body_id[i - 1], Math::Xtrans(joint_pos[i]), joint[i], body[i]);
	}
}

//Receives the position of the end effector
Vector3d Robot_model::getPosition()
{
	Vector3d pos;
	pos = CalcBodyToBaseCoordinates(*model, Q, body_id[dof_ - 1], com_pos[dof_ - 1], true);
	return pos;
}
Vector3d Robot_model::getPosition2(VectorXd q)
{
	Vector3d pos;
	Math::VectorNd Q2 = q;
	pos = CalcBodyToBaseCoordinates(*model, Q2, body_id[dof_ - 1], com_pos[dof_ - 1], true);
	return pos;
}

Matrix3d Robot_model::getOrientation()
{
	Matrix3d Rot;
	Matrix3d Rot2;
	Rot2.setZero();
	Rot2(0, 0) = 1.0;
	Rot2(2, 1) = 1.0;
	Rot2(1, 2) = -1.0;

	Rot = CalcBodyWorldOrientation(*model, Q, body_id[dof_ - 1], true).transpose();
	Rot = Rot * Rot2;
	return Rot;
}


Matrix3d Robot_model::getOrientation2(VectorXd q2)
{
	Matrix3d Rot;
	Math::VectorNd Q2 = q2;
	Matrix3d Rot2;
	Rot2.setZero();
	Rot2(0, 0) = 1.0;
	Rot2(2, 1) = 1.0;
	Rot2(1, 2) = -1.0;

	Rot = CalcBodyWorldOrientation(*model, Q2, body_id[dof_ - 1], true).transpose();
	return Rot*Rot2;
}
MatrixXd Robot_model::getJacobian()
{
	MatrixXd H2(6, dof_);

	CalcPointJacobian6D(*model, Q, body_id[dof_ - 1], com_pos[dof_ - 1], H2, true);
	MatrixXd J(6, dof_);
	for (int i = 0; i < 2; i++) {
		J.block(i * 3, 0, 3, dof_) = H2.block(3 - i * 3, 0, 3, dof_);
	}

	return J;
}
MatrixXd Robot_model::getJacobian2(VectorXd q2)
{
	MatrixXd H2(6, dof_);
	Math::VectorNd Q2 = q2;

	CalcPointJacobian6D(*model, Q2, body_id[dof_ - 1], com_pos[dof_ - 1], H2, true);
	MatrixXd J(6, dof_);
	for (int i = 0; i < 2; i++) {
		J.block(i * 3, 0, 3, dof_) = H2.block(3 - i * 3, 0, 3, dof_);
	}

	return J;
}
Vector3d Robot_model::GetPhi(Matrix3d Rot, Matrix3d Rotd)
{
	Vector3d phi;
	Vector3d s[3], v[3], w[3];

	for (int i = 0; i < 3; i++) {
		v[i] = Rot.block(0, i, 3, 1);
		w[i] = Rotd.block(0, i, 3, 1);
		s[i] = v[i].cross(w[i]);
	}
	phi = s[0] + s[1] + s[2];
	phi = -0.5* phi;

	return phi;
}

VectorXd Robot_model::solveIK1()
{
	VectorXd q(dof_);
	q.setZero();

	getPosition();
	return q;

}

bool checkRealNumber(VectorXd q) {
	bool truth = true;
	for (int i = 0; i < 6; i++) {
		if (_isnan(q(i))) { //if it is not a real number
			truth = false;
			break;
		}
	}
	return truth;
}

void printAngle(VectorXd q) {
	cout << "각도 출력" << endl;
	cout << q << endl << endl;
	cout << q * 180 / M_PI << endl<<endl;
}

//Parameter : End effector's location, orientation
VectorXd Robot_model::analyticIKSolver(Vector3d x_d, Matrix3d ori_d)
{
	T06f << ori_d, x_d,
		0, 0, 0, 1;
	cout << T06f<<endl;
	VectorXd* q_answer = new VectorXd(dof_);
	q_answer->setZero();
	Vector3d p05(3);
	p05.setZero();
	p05(0) = x_d(0) - ori_d(0, 2) * d6;
	p05(1) = x_d(1) - ori_d(1, 2) * d6;
	p05(2) = x_d(2) - ori_d(2, 2) * d6;
	for (int i = 1; i < 3; i++) {
		for (int j = 1; j < 3; j++) {
			for (int k = 1; k < 3; k++) {
				(*q_answer)(0) = atan2(p05(1), p05(0)) + pow(-1, i)*acos(d4 / sqrt(pow(p05(0), 2) + pow(p05(1), 2))) + M_PI / 2;

				(*q_answer)(4) = pow(-1, j) * acos((x_d(0)*sin((*q_answer)(0)) - x_d(1)*cos((*q_answer)(0)) - d4) / d6);
				if ((!_isnan(T06f(0, 1)*sin((*q_answer)(0)) + T06f(1, 1)*cos((*q_answer)(0)) / sin((*q_answer)(4)))) && (!_isnan(T06f(0, 0)*sin((*q_answer)(0)) - T06f(1, 0)*cos((*q_answer)(0)) / sin((*q_answer)(4)))))
					(*q_answer)(5) = atan2((-T06f(0, 1)*sin((*q_answer)(0)) + T06f(1, 1)*cos((*q_answer)(0))) / sin((*q_answer)(4)), (T06f(0, 0)*sin((*q_answer)(0)) - T06f(1, 0)*cos((*q_answer)(0))) / sin((*q_answer)(4)));
				else
					(*q_answer)(5) = 0;

				forQ << cos((*q_answer)(0)), -sin(((*q_answer)(0))), 0, 0,
					sin((*q_answer)(0)), cos(((*q_answer)(0))), 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
				T02f = eye * eye*forQ*D1;
				forQ << cos((*q_answer)(4)), -sin(((*q_answer)(4))), 0, 0,
					sin((*q_answer)(4)), cos(((*q_answer)(4))), 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
				T45f = temp1 * eye * forQ * D5;
				forQ << cos((*q_answer)(5)), -sin(((*q_answer)(5))), 0, 0,
					sin((*q_answer)(5)), cos(((*q_answer)(5))), 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
				T56f = temp2 * eye * forQ * D6;
				T46f = T45f * T56f;
				T24f = T02f.inverse() * T06f * T46f.inverse();
				(*q_answer)(2) = pow(-1, k)*acos((T24f(0, 3)*T24f(0, 3) + T24f(2, 3)*T24f(2, 3) - a2 * a2 - a3 * a3) / (2 * a2*a3));

				(*q_answer)(1) = atan2(T24f(2, 3), T24f(0, 3)) - asin(a3*sin((*q_answer)(2)) / sqrt(T24f(0, 3)*T24f(0, 3) + T24f(2, 3)*T24f(2, 3)));
				R02f = T02f.block(0, 0, 3, 3);
				R46f = T46f.block(0, 0, 3, 3);
				R24f = temp3.transpose()* R02f.transpose() * T06f.block(0, 0, 3, 3) *R46f.transpose();
				(*q_answer)(3) = atan2(R24f(1, 0), R24f(0, 0)) - (*q_answer)(1) - (*q_answer)(2);
				if (checkRealNumber((*q_answer))) {
					printAngle((*q_answer));
				}
			}
		}
	}
	return (*q_answer);
}
VectorXd Robot_model::JacobianBasedSolver(Vector3d x_d, Matrix3d ori_d)
{
	int MaxIter = 10000;
	int iter = 0;
	double tol = 100.0, threshold = 1e-6;
	VectorXd* q_answer = new VectorXd(dof_);
	VectorXd q_prev(dof_), q_dot(dof_), x_dot(dof_), x_error(dof_);
	Vector3d x_current;
	Matrix3d Rot_current;

	MatrixXd J_temp(6, 6), J_inv(6, 6);
	MatrixXd eye(6, 6);
	eye.setIdentity();

	double kp = 10.0, kv = 0.01;
	q_answer->setZero();
	J_temp.resize(6, 6);
	q_prev = *q_answer;

	while (iter < MaxIter) {
		x_current = getPosition2(*q_answer);
		Rot_current = getOrientation2(*q_answer);
		J_temp = getJacobian2(*q_answer);
		//J_inv = J_temp.transpose() * (J_temp * J_temp.transpose() + 0.001 * eye).inverse();
		J_inv = J_temp.completeOrthogonalDecomposition().pseudoInverse();
		x_error.tail(3) = -1.0 * GetPhi(Rot_current, ori_d);
		x_error.head(3) = x_d - x_current;

		q_dot = (*q_answer - q_prev) * 100.0;
		q_prev = *q_answer;
		x_dot = J_temp * q_dot;
		*q_answer += J_inv * (kp * x_error + kv * x_dot) / 100.0;
		tol = x_error.norm();

		if (tol < threshold) {
			cout << "IK Solved" << endl << endl;
			break;
		}
		iter++;
	}

	if (iter != MaxIter) {
		cout << "Solved" << endl << endl;
		cout << "Iter " << iter << " " << " " << "tol " << tol << endl << endl;
		cout << "answer" << endl << *q_answer * 180.0 / M_PI << endl << endl;
	}
	else {
		cout << "IK Failed" << endl;
	}

	return *q_answer;
}