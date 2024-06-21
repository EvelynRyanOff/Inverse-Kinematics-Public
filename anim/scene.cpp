#include "scene.h"

scene::scene(const std::string& name) : BaseSystem(name) {

}

// TODO: these must be overloaded by any derived class
void scene::getState(double* p) {
	return;
}
void scene::setState(double* p) {
	return;
}
void scene::display(GLenum mode) {
	
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
//	glTranslated(m_pos[0], m_pos[1], m_pos[2]);
	glScalef(1, 1, 1);

	glColor3f(1, 1, 1);
	glBegin(GL_TRIANGLES);
	glVertex3f(-22, 22, -0.5);
	glVertex3f(-22, -22, -0.5);
	glVertex3f(22, 22, -0.5);
	glVertex3f(22, 22, -0.5);
	glVertex3f(-22, -22, -0.5);
	glVertex3f(22, -22, -0.5);
	glEnd();

	glColor3f(0, 0, 0);

	glBegin(GL_TRIANGLES);
	glVertex3f(-10, 7, -0.25);
	glVertex3f(-10, -7, -0.25);
	glVertex3f(10, 7, -0.25);
	glVertex3f(10, 7, -0.25);
	glVertex3f(-10, -7, -0.25);
	glVertex3f(10, -7, -0.25);
	glEnd();

	glColor3f(1, 0.89, 0.16);
	glBegin(GL_TRIANGLES);
	glVertex3f(-22, -22, -0.5);
	glVertex3f(22, -22, -0.5);
	glVertex3f(22, -22, 20);
	glVertex3f(-22, -22, -0.5);
	glVertex3f(22, -22, 20);
	glVertex3f(-22, -22, 20);
	glEnd();


	glPopMatrix();
	glPopAttrib();


	
	
}


