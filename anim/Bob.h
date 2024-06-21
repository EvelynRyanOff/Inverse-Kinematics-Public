#pragma once
#ifndef MY_BOB_H
#define MY_BOB_H

#ifdef Success 
#undef Success
#endif

#include "lib/Eigen/Dense"

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include <math.h>
#include "string.h"
#include "shared/opengl.h"
#include <iostream>
#include <string>
#include <sstream>

typedef std::shared_ptr<class Bob> BobRef;

class Bob : public BaseSystem
{

public:
	Bob(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void display(GLenum mode = GL_RENDER);

	void __init_angles();

	//Matrices for F and Jaccobian
	Eigen::MatrixXd Rx(double theta);
	Eigen::MatrixXd dRx(double theta);

	Eigen::MatrixXd Ry(double theta);
	Eigen::MatrixXd dRy(double theta);

	Eigen::MatrixXd Rz(double theta);
	Eigen::MatrixXd dRz(double theta);

	Eigen::MatrixXd Troot();
	Eigen::MatrixXd Tsh();
	Eigen::MatrixXd Tel();
	Eigen::MatrixXd Twr();
	//end of matrices
	//function which assembles the jacobian
	Eigen::MatrixXd jacobian();

	Eigen::MatrixXd get_phand();
	Eigen::MatrixXd current_pos();
	void IK_solver(Vector target_point); //angels and starting point are saved in memory
	void IK_solve(Eigen::MatrixXd target, Eigen::MatrixXd current);
	void _IK_solve(Eigen::MatrixXd theta);
	int command(int argc, myCONST_SPEC char** argv);

	double left_angels[7];

protected:

	

	Vector m_pos;
	int init;

	//double left_angels[7];
	double right_angels[2];
	

};
#endif