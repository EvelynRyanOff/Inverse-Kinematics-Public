#include "Iksim.h"





Iksim::Iksim( const std::string& name, HermiteRef target1, BobRef target2):
	BaseSimulator( name ),
	hermite( target1 ),
	bob( target2 )
{
	t = 0;
	in = 0;
}	// SampleGravitySimulator

Iksim::~Iksim()
{
}	// SampleGravitySimulator::~SampleGravitySimulator

void Iksim::load_process(myCONST_SPEC char** argv) {
	
	VectorObj p;
	argv[0] = "load2D";
	
	hermite->command(2, argv);
	p = hermite->getIntermediatePoint(0);
	m_pos[0]  = p[0];
	m_pos[1]  = p[1];
	m_pos[2]  = p[2];

	char** arg = new char* [4];

	
	std::stringstream strs;
	strs << p[0];
	string temp_str = strs.str();
	char* char_type = (char*)temp_str.c_str();
	arg[1] = char_type;

	std::stringstream strs1;
	strs1 << p[1];
	string temp_str1 = strs1.str();
	char* char_type1 = (char*)temp_str1.c_str();
	arg[2] = char_type1;
	
	std::stringstream strs2;
	strs2 << p[2];
	string temp_str2 = strs2.str();
	char* char_type2 = (char*)temp_str2.c_str();
	arg[3] = char_type2;

	arg[0] = "load";
	bob->command(4, arg);


}


int Iksim::command(int argc, myCONST_SPEC char** argv) {
	if (argc == 0) {
		return TCL_OK;
	}
	else if (strcmp(argv[0], "read") == 0 && argc == 2) {
		in = 1;
		load_process(argv);

		return TCL_OK;
	}

	return TCL_OK;

}

int Iksim::step(double time)
{	
	if (t >= 1) {
		t = 0;
	}
	if (in == 1) {
		
		Eigen::MatrixXd pos_m(4, 1);
		Vector pos;
		Vector target;
		Vector traj;
		pos_m = bob->current_pos();
		pos[0] = pos_m(0, 0);
		pos[1] = pos_m(1, 0);
		pos[2] = pos_m(2, 0);

		VecCopy(target,m_pos);
		

		traj[0] = target[0] - pos[0];
		traj[1] = target[1] - pos[1];
		traj[2] = target[2] - pos[2];

		double length;
		length = sqrt(pow(traj[0],2)+pow(traj[1],2)+pow(traj[2],2));
		
		if (length > 0.1) {
			
			traj[0] = traj[0]/length;
			traj[1] = traj[1]/length;
			traj[2] = traj[2]/length;


			traj[0] = 0.1 * traj[0];
			traj[1] = 0.1 * traj[1];
			traj[2] = 0.1 * traj[2];

			target[0] = pos[0] + traj[0];
			target[1] = pos[1] + traj[1];
			target[2] = pos[2] + traj[2];

			

			bob->IK_solver(target);


		}
		else {

			bob->IK_solver(target);
			VectorObj p;
			t += 0.1;
			if (t <= 1) {
				p = hermite->getIntermediatePoint(t);

				m_pos[0] = p[0];
				m_pos[1] = p[1];
				m_pos[2] = p[2];
			}


		}


	}
	
	return 0;

}	
