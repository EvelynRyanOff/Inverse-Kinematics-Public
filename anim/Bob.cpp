#include "Bob.h"

Bob::Bob(const std::string& name) :BaseSystem(name) {
	setVector(m_pos, 0, 0, 0);

	right_angels[0] = 0;
	right_angels[1] = 0;
	
	left_angels[0] = 0;
	left_angels[1] = 0;
	left_angels[2] = 0;
	left_angels[3] = 0;
	left_angels[4] = 0;
	left_angels[5] = 0;
	left_angels[6] = 0;

	init = 0;

}
void Bob::getState(double* p) {

}
void Bob::setState(double* p) {

}
void Bob::reset(double time) {

}

Eigen::MatrixXd Bob::Rx(double theta) {
	Eigen::MatrixXd m(4, 4);
	m << 1, 0, 0, 0,
		0, cos(theta*(PI/180)), -1*sin(theta*(PI/180)), 0,
		0, sin(theta*(PI/180)), cos(theta*(PI/180)), 0,
		0, 0, 0, 1;


	return m;
}
Eigen::MatrixXd Bob::dRx(double theta) {
	Eigen::MatrixXd m(4, 4);
	m << 0, 0, 0, 0,
		0, -1*sin(theta*(PI/180)), -1*cos(theta*(PI/180)), 0,
		0, cos(theta*(PI/180)), -1*sin(theta*(PI/180)), 0,
		0, 0, 0, 0;


	return m;
}
Eigen::MatrixXd Bob::Ry(double theta) {
	Eigen::MatrixXd m(4, 4);
	m << cos(theta * (PI / 180)), 0, sin(theta * (PI / 180)), 0,
		0, 1, 0, 0,
		-1 * sin(theta * (PI / 180)), 0, cos(theta * (PI / 180)), 0,
		0, 0, 0, 1;


	return m;
}
Eigen::MatrixXd Bob::dRy(double theta) {
	Eigen::MatrixXd m(4, 4);
	m << -1 * sin(theta * (PI / 180)), 0, cos(theta * (PI / 180)), 0,
		0, 0, 0, 0,
		-1 * cos(theta * (PI / 180)), 0, -1 * sin(theta * (PI / 180)), 0,
		0, 0, 0, 0;


	return m;
}
Eigen::MatrixXd Bob::Rz(double theta) {
	Eigen::MatrixXd m(4, 4);
	m << cos(theta * (PI / 180)), -1 * sin(theta * (PI / 180)), 0, 0,
		sin(theta * (PI / 180)), cos(theta * (PI / 180)), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;


	return m;
}
Eigen::MatrixXd Bob::dRz(double theta) {
	Eigen::MatrixXd m(4, 4);
	m << -1*sin(theta * (PI / 180)), -1 * cos(theta * (PI / 180)), 0, 0,
		cos(theta * (PI / 180)), -1*sin(theta * (PI / 180)), 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;


	return m;
}
Eigen::MatrixXd Bob::Troot() {
	Eigen::MatrixXd m(4, 4);
	m << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 13,
		0, 0, 0, 1;

	return m;
}Eigen::MatrixXd Bob::Tsh() {
	Eigen::MatrixXd m(4, 4);
	m << 1, 0, 0, -6,
		0, 1, 0, 3,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return m;
}Eigen::MatrixXd Bob::Tel() {
	Eigen::MatrixXd m(4, 4);
	m << 1, 0, 0, -7,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return m;
}Eigen::MatrixXd Bob::Twr() {
	Eigen::MatrixXd m(4, 4);
	m << 1, 0, 0, -7,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return m;
}
Eigen::MatrixXd Bob::jacobian() {
	//p is of size 4x1
	Eigen::MatrixXd p(4, 1);
	p = get_phand();
	Eigen::MatrixXd J(4, 7);
	Eigen::MatrixXd dF(4, 1);
	Eigen::MatrixXd next_matrix(4, 4);

	dF = Troot() * Tsh() * Rz(left_angels[2]) * Ry(left_angels[1]) * dRx(left_angels[0]) * Tel() * Ry(left_angels[4]) * Rx(left_angels[3]) * Twr() * Ry(left_angels[6]) * Rz(left_angels[5]) * p;
	J(0,0) = dF(0,0);
	J(1, 0) = dF(1,0);
	J(2, 0) = dF(2, 0);
	J(3, 0) = 0;

	dF = Troot() * Tsh() * Rz(left_angels[2]) * dRy(left_angels[1]) * Rx(left_angels[0]) * Tel() * Ry(left_angels[4]) * Rx(left_angels[3]) * Twr() * Ry(left_angels[6]) * Rz(left_angels[5]) * p;
	J(0, 1) = dF(0, 0);
	J(1, 1) = dF(1, 0);
	J(2, 1) = dF(2, 0);
	J(3, 1) = 0;
	
	dF = Troot() * Tsh() * dRz(left_angels[2]) * Ry(left_angels[1]) * Rx(left_angels[0]) * Tel() * Ry(left_angels[4]) * Rx(left_angels[3]) * Twr() * Ry(left_angels[6]) * Rz(left_angels[5]) * p;
	J(0, 2) = dF(0, 0);
	J(1, 2) = dF(1, 0);
	J(2, 2) = dF(2, 0);
	J(3, 2) = 0;
	
	dF = Troot() * Tsh() * Rz(left_angels[2]) * Ry(left_angels[1]) * Rx(left_angels[0]) * Tel() * Ry(left_angels[4]) * dRx(left_angels[3]) * Twr() * Ry(left_angels[6]) * Rz(left_angels[5]) * p;
	J(0, 3) = dF(0, 0);
	J(1, 3) = dF(1, 0);
	J(2, 3) = dF(2, 0);
	J(3, 3) = 0;

	dF = Troot() * Tsh() * Rz(left_angels[2]) * Ry(left_angels[1]) * Rx(left_angels[0]) * Tel() * dRy(left_angels[4]) * Rx(left_angels[3]) * Twr() * Ry(left_angels[6]) * Rz(left_angels[5]) * p;
	J(0, 4) = dF(0, 0);
	J(1, 4) = dF(1, 0);
	J(2, 4) = dF(2, 0);
	J(3, 4) = 0;

	dF = Troot() * Tsh() * Rz(left_angels[2]) * Ry(left_angels[1]) * Rx(left_angels[0]) * Tel() * Ry(left_angels[4]) * Rx(left_angels[3]) * Twr() * Ry(left_angels[6]) * dRz(left_angels[5]) * p;
	J(0, 5) = dF(0, 0);
	J(1, 5) = dF(1, 0);
	J(2, 5) = dF(2, 0);
	J(3, 5) = 0;

	dF = Troot() * Tsh() * Rz(left_angels[2]) * Ry(left_angels[1]) * Rx(left_angels[0]) * Tel() * Ry(left_angels[4]) * Rx(left_angels[3]) * Twr() * dRy(left_angels[6]) * Rz(left_angels[5]) * p;
	J(0, 6) = dF(0, 0);
	J(1, 6) = dF(1, 0);
	J(2, 6) = dF(2, 0);
	J(3, 6) = 1;


	return J;
}
Eigen::MatrixXd Bob::current_pos() {
	Eigen::MatrixXd m(4, 1);
	m(0, 0) = -4;
	m(1, 0) = 0;
	m(2, 0) = 0;
	m(3, 0) = 1;
	return Troot() * Tsh() * Rz(left_angels[2]) * Ry(left_angels[1]) * Rx(left_angels[0]) * Tel() * Ry(left_angels[4]) * Rx(left_angels[3]) * Twr() * Ry(left_angels[6]) * Rz(left_angels[5]) * m;
}

Eigen::MatrixXd Bob::get_phand() {
	Eigen::MatrixXd phand(4, 1);

	phand(0, 0) = -4;
	phand(1, 0) = 0;
	phand(2, 0) = 0;
	phand(3, 0) = 1;
	
	return phand;
}


void Bob::display(GLenum mode) {

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glMatrixMode(GL_MODELVIEW);
	glColor3f(0, 1, 0);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	

	//line drawing here
	Eigen::MatrixXd m(4, 1);
	m = current_pos();
	glPointSize(10);
	glBegin(GL_POINTS);
	glColor3f(1, 0, 0);
	glVertex3d(m(0,0), m(1,0),m(2,0));

	if (init == 1) {
		glVertex3d(m_pos[0], m_pos[1], m_pos[2]);
		glEnd();
		glColor3f(0, 1, 0);
		glBegin(GL_LINE_STRIP);

		glVertex3d(m(0, 0), m(1, 0), m(2, 0));
		glVertex3d(m_pos[0], m_pos[1], m_pos[2]);

		glEnd();
	}else { 
		glEnd(); 
		glColor3f(0, 1, 0);
	}
	//line drawing end

	glTranslated(0, 0, 13);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glScalef(1, 1.5, 0.5);

	
	glutWireSphere(5,20, 20);
	glPopMatrix();
	glPopAttrib();

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//head
	glTranslated(0, 9, 0);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	
	glutWireSphere(3.5, 20, 20);

	glPopMatrix();
	glPopAttrib();

	glPopMatrix();
	glPopAttrib();
	//head end
	// 
	//left arm start
	//this will be the animated arm
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//shoulder, three degrees of freedom


	glTranslated(-6, 3, 0);
	//glutWireSphere(18, 20, 20);
	
	
	glRotated(left_angels[2], 0.0, 0.0, 1.0);
	glRotated(left_angels[1], 0.0, 1.0, 0.0);
	glRotated(left_angels[0], 1.0, 0.0, 0.0);
	

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(-2.5, 0, 0);
	glScalef(1, 0.5, 0.5);
	glutWireSphere(4, 20, 20);

	glPopMatrix();
	glPopAttrib();

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//elbow, two degrees of freedom: x y
	glTranslated(-7, 0, 0);
	
	
	glRotated(left_angels[4], 0.0, 1.0, 0.0);
	glRotated(left_angels[3], 1.0, 0.0, 0.0);
	
	


	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(-3, 0, 0);
	glScalef(1, 0.5, 0.5);
	glutWireSphere(4, 20, 20);

	glPopMatrix();
	glPopAttrib();

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//hand, two degrees of freedom: y,z
	glTranslated(-7, 0, 0);
	
	
	glRotated(left_angels[5], 0.0, 1.0, 0.0);
	glRotated(left_angels[6], 0.0, 0.0, 1.0);
	


	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(-1.5, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	glutWireSphere(4, 20, 20);
	glPopMatrix();
	glPopAttrib();

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//end affector
	glColor3f(0, 0, 0);
	glTranslated(-4, 0, 0);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPointSize(10);
	glBegin(GL_POINTS);
	glColor3f(1, 0, 0);
	glVertex3d(0, 0, 0);
	glEnd();
	glColor3f(0, 1, 0);
	
	//glScalef(1, 1, 1);
	//glutWireSphere(1, 20, 20);
	glPopMatrix();
	glPopAttrib();
	
	glColor3f(0, 1, 0);



	glPopMatrix();
	glPopAttrib();
	//end affector end
	glPopMatrix();
	glPopAttrib();
	//hand end
	glPopMatrix();
	glPopAttrib();
	//segment2 end
	glPopMatrix();
	glPopAttrib();
	//left arm end

	//right arm start
	//non animated arm
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//segment 1
	
	glTranslated(6, 3, 0);
	glRotated(right_angels[0] , 0.0, 0.0, 1.0);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(2.5, 0, 0);
	glScalef(1, 0.5, 0.5);
	glutWireSphere(4, 20, 20);
	glPopMatrix();
	glPopAttrib();


	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//segment2
	glTranslated(7, 0, 0);
	glRotated(right_angels[1], 0.0, 0.0, 1.0);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(3, 0, 0);
	glScalef(1, 0.5, 0.5);
	glutWireSphere(4, 20, 20);

	glPopMatrix();
	glPopAttrib();


	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//hand
	glTranslated(7, 0, 0);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(1.5, 0, 0);
	glScalef(0.5, 0.5, 0.5);
	glutWireSphere(4, 20, 20);
	glPopMatrix();
	glPopAttrib();

	glPopMatrix();
	glPopAttrib();
	//hand end
	glPopMatrix();
	glPopAttrib();
	//segment2 end
	
	glPopMatrix();
	glPopAttrib();
	//right arm end

	//left leg begin
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//segment 1 begin
	glTranslated(-4, -9, 0);
	
	glScalef(0.5, 1, 0.5);
	glutWireSphere(4, 20, 20);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//leg l seg 2 begin

	glTranslated(0, -8, 0);
	glutWireSphere(4, 20, 20);
	

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//leg l foot begin



	glTranslated(0, -4, 0);
	glScalef(1/0.5, 1, 1);
	glScalef(0.5, 0.25, 1);
	glutWireSphere(4, 20, 20);


	glPopMatrix();
	glPopAttrib();
	//leg l foot end

	glPopMatrix();
	glPopAttrib();
	//leg l seg 2 end


	glPopMatrix();
	glPopAttrib();
	// leg l segment 1 end

	//right leg begin
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//segment 1 begin
	glTranslated(4, -9, 0);
	//glScalef(1, 1 / 1.5, 1 / 0.5);
	glScalef(0.5, 1, 0.5);
	glutWireSphere(4, 20, 20);

	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//leg r seg 2 begin

	glTranslated(0, -8, 0);
	glutWireSphere(4, 20, 20);


	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//leg r foot begin



	glTranslated(0, -4, 0);
	glScalef(1 / 0.5, 1, 1);
	glScalef(0.5, 0.25, 1);
	glutWireSphere(4, 20, 20);


	glPopMatrix();
	glPopAttrib();
	//leg r foot end

	glPopMatrix();
	glPopAttrib();
	//leg r seg 2 end


	glPopMatrix();
	glPopAttrib();
	// leg r segment 1 end




	glPopMatrix();
	glPopAttrib();

}

void Bob::__init_angles() {

	right_angels[0] = -30.0;
	right_angels[1] = -90.0;

	left_angels[0] = 10;
	left_angels[1] = 3;

	left_angels[2] = 70;
	left_angels[3] = -2;
	left_angels[4] = -90;
	left_angels[5] = -1;
	left_angels[6] = -2;

	return;

}
void Bob::_IK_solve(Eigen::MatrixXd theta) {
	/*
	std::stringstream s;
	s << theta;
	
	std::string temp_str = s.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	*/
	left_angels[0] = left_angels[0] + theta(0,0);
	left_angels[1] = left_angels[1] + theta(1,0);
	left_angels[2] = left_angels[2] + theta(2,0);
	left_angels[3] = left_angels[3] + theta(3,0);
	left_angels[4] = left_angels[4] + theta(4,0);
	left_angels[5] = left_angels[5] + theta(5,0);
	left_angels[6] = left_angels[6] + theta(6,0);




}

void Bob::IK_solve(Eigen::MatrixXd target, Eigen::MatrixXd current) {

	Eigen::MatrixXd x_dot(4,1);
	x_dot = target - current;
	//potential problem: Jaccobian maybe should be 4x8
	Eigen::MatrixXd J(4,7);
	Eigen::MatrixXd J_t(7,4);
	Eigen::MatrixXd J_product(4,4);

	J = jacobian();
	J_t = J.transpose();
	J_product = J * J_t;
	
	
	Eigen::MatrixXd beta(4, 1);
	beta = (J_product.lu()).solve(x_dot);
	
	Eigen::MatrixXd theta (7, 1);
	theta = J_t*beta;
	/*
	std::stringstream s;
	s << J_product.lu().matrixLU();
	std::string temp_str = s.str();
	char* char_type = (char*)temp_str.c_str();
	animTcl::OutputMessage(char_type);
	*/
	_IK_solve(theta);
	
}

void Bob::IK_solver(Vector target_point) {
	double error_norm = 100;

	while (error_norm > 0.001) {
		Eigen::MatrixXd ptarget(4, 1);
		Eigen::MatrixXd shoulder(4, 1);
		ptarget(0, 0) = target_point[0];
		ptarget(1, 0) = target_point[1];
		ptarget(2, 0) = target_point[2];
		ptarget(3, 0) = 1;

		shoulder(0, 0) = -6;
		shoulder(1, 0) = 3;
		shoulder(2, 0) = 13;
		shoulder(3, 0) = 1;



		Eigen::MatrixXd currentp(4, 1);
		currentp = current_pos();

		Eigen::MatrixXd err(4, 1);
		err = ptarget - currentp;

		Eigen::MatrixXd ptargetp(4, 1);
		ptargetp = (0.1 * err) + currentp;
	
		IK_solve(ptargetp, currentp);

		error_norm = err.norm();
		
		if ((shoulder - ptarget).norm() > 20) {
			animTcl::OutputMessage("out of range!!");
			return;
		}

	}
	return;
}


int Bob::command(int argc, myCONST_SPEC char** argv) {
	
	if (argc == 0) { return TCL_OK; }
	else if (strcmp(argv[0], "load") == 0) {
		if (argc == 4) {
			std::stringstream s;
			__init_angles();
			/*
			s << jacobian();
			std::string temp_str = s.str();
			char* char_type = (char*)temp_str.c_str();
			animTcl::OutputMessage(char_type);*/
		
			m_pos[0] = std::stod(argv[1]);
			m_pos[1] = std::stod(argv[2]);
			m_pos[2] = std::stod(argv[3]);

			init = 1;

			

		}
	}
	else if (strcmp(argv[0],"position") == 0) {
		if (argc == 4) {

			Vector target_point;
			target_point[0] = std::stod(argv[1]);
			target_point[1] = std::stod(argv[2]);
			target_point[2] = std::stod(argv[3]);

			IK_solver(target_point);
			
		}
	}
	

	return TCL_OK;
}