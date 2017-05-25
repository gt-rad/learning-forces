//
//  opengl.h
//  MassSpringLocomotion
//
//  Created by YuWenhao on 4/15/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef MassSpringLocomotion_opengl_h
#define MassSpringLocomotion_opengl_h

#if __APPLE__ & __MACH__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>
#include <GLUT/glut.h>
#elif defined(_WIN32)
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glx.h>
#include <GL/glext.h>
#include <GL/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <GL/glext.h>
#include <GL/freeglut.h>
#endif

#endif
