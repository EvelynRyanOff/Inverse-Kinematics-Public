#ifndef MY_GRAVITY_SIMULATOR_H
#define MY_GRAVITY_SIMULATOR_H


#ifdef Success 
#undef Success
#endif
#include "lib/Eigen/Dense"
#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Hermite.h"
#include <iostream>
#include <sstream>
#include <string>
#include <string.h>
#include "Bob.h"
#include <math.h>


// a sample simulator

class Iksim : public BaseSimulator 
{
public:

	Iksim( const std::string& name, HermiteRef target1, BobRef target2 );
	~Iksim();

	int step(double time);
	int init(double time) 
	{ 
		
		return 0;
	};
	void load_process(myCONST_SPEC char** argv);

	int command(int argc, myCONST_SPEC char **argv);

protected:

	Vector m_pos;
	Vector m_vel;
	int in;
	double t;

	
	HermiteRef hermite;
	BobRef bob;

};


#endif