#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <numeric>
#include <iostream>
#include <Windows.h>
#include <gl/GL.h>

#include "glut/glut.h"
#include "glm.h"
#include "mtxlib.h"
#include "trackball.h"

using namespace std;

_GLMmodel *ex_mesh;
int WindWidth, WindHeight;

int ex_last_x , ex_last_y;
int ex_selectedFeature = -1;
vector<int> ex_featureList;

void ex_Reshape(int width, int height)
{
  int base = min(width , height);

  tbReshape(width, height);
  glViewport(0 , 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0,(GLdouble)width / (GLdouble)height , 1.0, 128.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -3.5);

  WindWidth = width;
  WindHeight = height;
}

void ex_Display(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glPushMatrix();
  tbMatrix();
  
  // render solid model
  glEnable(GL_LIGHTING);
  glColor3f(1.0 , 1.0 , 1.0f);
  glPolygonMode(GL_FRONT_AND_BACK , GL_FILL);
  glmDraw(ex_mesh , GLM_SMOOTH);

  // render wire model
  glPolygonOffset(1.0 , 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glLineWidth(1.0f);
  glColor3f(0.6 , 0.0 , 0.8);
  glPolygonMode(GL_FRONT_AND_BACK , GL_LINE);
  glmDraw(ex_mesh , GLM_SMOOTH);

  // render features
  glPointSize(10.0);
  glColor3f(1.0 , 0.0 , 0.0);
  glDisable(GL_LIGHTING);
  glBegin(GL_POINTS);
	for (int i = 0 ; i < ex_featureList.size() ; i++)
	{
		int idx = ex_featureList[i];

		glVertex3fv((float *)&ex_mesh->vertices[3 * idx]);
	}
  glEnd();
  
  glPopMatrix();

  glFlush();  
  glutSwapBuffers();
}

vector3 ex_Unprojection(vector2 _2Dpos)
{
	float Depth;
	int viewport[4];
	double ModelViewMatrix[16];				//Model_view matrix
	double ProjectionMatrix[16];			//Projection matrix

	glPushMatrix();
	tbMatrix();

	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

	glPopMatrix();

	glReadPixels((int)_2Dpos.x , viewport[3] - (int)_2Dpos.y , 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

	double X = _2Dpos.x;
	double Y = _2Dpos.y;
	double wpos[3] = {0.0 , 0.0 , 0.0};

	gluUnProject(X , ((double)viewport[3] - Y) , (double)Depth , ModelViewMatrix , ProjectionMatrix , viewport, &wpos[0] , &wpos[1] , &wpos[2]);

	return vector3(wpos[0] , wpos[1] , wpos[2]);
}

void ex_mouse(int button, int state, int x, int y)
{
  tbMouse(button, state, x, y);

  // add feature
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = ex_Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < ex_mesh->numvertices ; i++)
	  {
		  vector3 pt(ex_mesh->vertices[3 * i + 0] , ex_mesh->vertices[3 * i + 1] , ex_mesh->vertices[3 * i + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = i;
		  }
	  }

	  ex_featureList.push_back(minIdx);
  }

  // manipulate feature
  if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
  {
	  int minIdx = 0;
	  float minDis = 9999999.0f;

	  vector3 pos = ex_Unprojection(vector2((float)x , (float)y));

	  for (int i = 0 ; i < ex_featureList.size() ; i++)
	  {
		  int idx = ex_featureList[i];
		  vector3 pt(ex_mesh->vertices[3 * idx + 0] , ex_mesh->vertices[3 * idx + 1] , ex_mesh->vertices[3 * idx + 2]);
		  float dis = (pos - pt).length();

		  if (minDis > dis)
		  {
			  minDis = dis;
			  minIdx = ex_featureList[i];
		  }
	  }

	  ex_selectedFeature = minIdx;
  }

  if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	  ex_selectedFeature = -1;

  ex_last_x = x;
  ex_last_y = y;
}

void ex_motion(int x, int y)
{
  tbMotion(x, y);

  if (ex_selectedFeature != -1)
  {
	  matrix44 m;
	  vector4 vec = vector4((float)(x - ex_last_x) / 100.0f , (float)(y - ex_last_y) / 100.0f , 0.0 , 1.0);
	  
	  gettbMatrix((float *)&m);
	  vec = m * vec;

	  ex_mesh->vertices[3 * ex_selectedFeature + 0] += vec.x;
	  ex_mesh->vertices[3 * ex_selectedFeature + 1] -= vec.y;
	  ex_mesh->vertices[3 * ex_selectedFeature + 2] += vec.z;
  }

  ex_last_x = x;
  ex_last_y = y;
}

void ex_timf(int value)
{
  glutPostRedisplay();
  glutTimerFunc(1, ex_timf, 0);
}

int ex_main(int argc, char *argv[])
{
  WindWidth = 400;
  WindHeight = 400;
	
  GLfloat light_ambient[] = {0.0, 0.0, 0.0, 1.0};
  GLfloat light_diffuse[] = {0.8, 0.8, 0.8, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position[] = {0.0, 0.0, 1.0, 0.0};

  glutInit(&argc, argv);
  glutInitWindowSize(WindWidth, WindHeight);
  glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow("Trackball Example");

  glutReshapeFunc(ex_Reshape);
  glutDisplayFunc(ex_Display);
  glutMouseFunc(ex_mouse);
  glutMotionFunc(ex_motion);
  glClearColor(0, 0, 0, 0);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glEnable(GL_LIGHT0);
  glDepthFunc(GL_LESS);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  tbInit(GLUT_LEFT_BUTTON);
  tbAnimate(GL_TRUE);

  glutTimerFunc(40, ex_timf, 0); // Set up timer for 40ms, about 25 fps

  // load 3D model
  ex_mesh = glmReadOBJ("../data/head.obj");
  
  glmUnitize(ex_mesh);
  glmFacetNormals(ex_mesh);
  glmVertexNormals(ex_mesh , 90.0);

  glutMainLoop();

  return 0;

}
