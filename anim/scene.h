#pragma once
#include <shared/defs.h>	// FIXME: this should be in animTcl.h
#include "anim.h"
#include "BaseSystem.h"
#include <util/VectorObj.h>
#include <assert.h>
#include <GL/glut.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>

#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>

#include "shared/opengl.h"

class scene : public BaseSystem
{
public:

	scene(const std::string& name);

	// TODO: these must be overloaded by any derived class

	virtual void getState(double* p);
	virtual void setState(double* p);
	void display(GLenum mode = GL_RENDER);

};