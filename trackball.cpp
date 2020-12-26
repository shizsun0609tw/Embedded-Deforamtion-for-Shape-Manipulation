/* 
 *  Simple trackball-like motion adapted (ripped off) from projtex.c
 *  (written by David Yu and David Blythe).  See the SIGGRAPH '96
 *  Advanced OpenGL course notes.
 *
 *
 *  Usage:
 *  
 *  o  call tbInit() in before any other tb call
 *  o  call tbReshape() from the reshape callback
 *  o  call tbMatrix() to get the trackball matrix rotation
 *  o  call tbStartMotion() to begin trackball movememt
 *  o  call tbStopMotion() to stop trackball movememt
 *  o  call tbMotion() from the motion callback
 *  o  call tbAnimate(GL_TRUE) if you want the trackball to continue 
 *     spinning after the mouse button has been released
 *  o  call tbAnimate(GL_FALSE) if you want the trackball to stop 
 *     spinning after the mouse button has been released
 *
 *  Typical setup:
 *
 *
 */

/* includes */
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <memory.h>
#include <Windows.h>
#include <numeric>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include "glut/glut.h"

#ifndef INCLUDE_GLM_H
#define INCLUDE_GLM_H
#include "glm.h"
#include "mtxlib.h"
#endif

#include "trackball.h"

#include "deformationGraph.h"

using namespace std;

/* globals */
static GLuint    tb_lasttime;
static GLfloat   tb_lastposition[3];

static GLfloat   tb_angle = 0.0;
static GLfloat   tb_axis[3];
static GLfloat   tb_transform[4][4];

static GLuint    tb_width;
static GLuint    tb_height;

static GLint     tb_button = -1;
static GLboolean tb_tracking = GL_FALSE;
static GLboolean tb_animate = GL_TRUE;

_GLMmodel* originMesh;
_GLMmodel* samplingMesh;

typedef enum { SELECT_MODE, DEFORM_MODE } ControlMode;
ControlMode current_mode = SELECT_MODE;

vector<vector<int>> handles;
vector<float*> colors;
int selected_handle_id = -1;
int select_x, select_y;
int last_x, last_y;
bool deform_mesh_flag = false;

DeformationGraph deformationGraph;

/* functions */
static void _tbPointToVector(int x, int y, int width, int height, float v[3])
{
  float d, a;

  /* project x, y onto a hemi-sphere centered within width, height. */
  v[0] = (2.0 * x - width) / width;
  v[1] = (height - 2.0 * y) / height;
  d = sqrt(v[0] * v[0] + v[1] * v[1]);
  v[2] = cos((3.14159265 / 2.0) * ((d < 1.0) ? d : 1.0));
  a = 1.0 / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] *= a;
  v[1] *= a;
  v[2] *= a;
}

static void _tbAnimate(void)
{
  glutPostRedisplay();
}

void _tbStartMotion(int x, int y, int button, int time)
{
  assert(tb_button != -1);

  tb_tracking = GL_TRUE;
  tb_lasttime = time;
  _tbPointToVector(x, y, tb_width, tb_height, tb_lastposition);
}

void _tbStopMotion(int button, unsigned time)
{
  assert(tb_button != -1);

  tb_tracking = GL_FALSE;

  if (time == tb_lasttime && tb_animate) {
    glutIdleFunc(_tbAnimate);
  } else {
    tb_angle = 0.0;
    if (tb_animate)
      glutIdleFunc(0);
  }
}

void tbAnimate(GLboolean animate)
{
  tb_animate = animate;
}

void tbInit(GLuint button)
{
  tb_button = button;
  tb_angle = 0.0;

  /* put the identity in the trackball transform */
  glPushMatrix();
  glLoadIdentity();
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
  glPopMatrix();
}

void tbMatrix()
{
  assert(tb_button != -1);

  glPushMatrix();
  glLoadIdentity();
  glRotatef(tb_angle, tb_axis[0], tb_axis[1], tb_axis[2]);
  glMultMatrixf((GLfloat *)tb_transform);
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
  glPopMatrix();

  glMultMatrixf((GLfloat *)tb_transform);
}

vector2 projection_helper(vector3 _3Dpos)
{
    int viewport[4];
    double ModelViewMatrix[16];    // Model_view matrix
    double ProjectionMatrix[16];   // Projection matrix

    glPushMatrix();
    tbMatrix();

    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

    glPopMatrix();

    double wpos[3] = { 0.0 , 0.0 , 0.0 };
    gluProject(-_3Dpos.x, _3Dpos.y, _3Dpos.z, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

    return vector2(wpos[0], (double)viewport[3] - wpos[1]);
}

void gettbMatrix(float *m)
{
	glPushMatrix();
	glLoadIdentity();
	glRotatef(tb_angle, tb_axis[0], tb_axis[1], tb_axis[2]);
	glMultMatrixf((GLfloat *)tb_transform);
	glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
	glPopMatrix();

	memcpy(m , tb_transform , 16 * sizeof(float));
}

void tbReshape(int width, int height)
{
  assert(tb_button != -1);

  tb_width  = width;
  tb_height = height;
}

void tbMouse(int button, int state, int x, int y)
{
  assert(tb_button != -1);

  if (state == GLUT_DOWN && button == tb_button)
    _tbStartMotion(x, y, button, glutGet(GLUT_ELAPSED_TIME));
  else if (state == GLUT_UP && button == tb_button)
    _tbStopMotion(button, glutGet(GLUT_ELAPSED_TIME));
}

void tbMotion(int x, int y)
{
  GLfloat current_position[3], dx, dy, dz;

  assert(tb_button != -1);

  if (tb_tracking == GL_FALSE)
    return;

  _tbPointToVector(x, y, tb_width, tb_height, current_position);

  /* calculate the angle to rotate by (directly proportional to the
     length of the mouse movement */
  dx = current_position[0] - tb_lastposition[0];
  dy = current_position[1] - tb_lastposition[1];
  dz = current_position[2] - tb_lastposition[2];
  tb_angle = 90.0 * sqrt(dx * dx + dy * dy + dz * dz);

  /* calculate the axis of rotation (cross product) */
  tb_axis[0] = tb_lastposition[1] * current_position[2] - 
               tb_lastposition[2] * current_position[1];
  tb_axis[1] = tb_lastposition[2] * current_position[0] - 
               tb_lastposition[0] * current_position[2];
  tb_axis[2] = tb_lastposition[0] * current_position[1] - 
               tb_lastposition[1] * current_position[0];

  /* reset for next time */
  tb_lasttime = glutGet(GLUT_ELAPSED_TIME);
  tb_lastposition[0] = current_position[0];
  tb_lastposition[1] = current_position[1];
  tb_lastposition[2] = current_position[2];

  /* remember to draw new position */
  glutPostRedisplay();
}

vector3 Unprojection(vector2 _2Dpos)
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

    glReadPixels((int)_2Dpos.x, viewport[3] - (int)_2Dpos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

    double X = _2Dpos.x;
    double Y = _2Dpos.y;
    double wpos[3] = { 0.0 , 0.0 , 0.0 };

    gluUnProject(X, ((double)viewport[3] - Y), (double)Depth, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

    return vector3(wpos[0], wpos[1], wpos[2]);
}

void init(void)
{
    tbInit(GLUT_LEFT_BUTTON);
    tbAnimate(GL_TRUE);

    originMesh = glmReadOBJ("../data/man.obj");
    samplingMesh = glmReadOBJ("../data/man_sampling.obj");
        
    glmUnitize(samplingMesh);
    glmUnitize(originMesh);
    glmFacetNormals(samplingMesh);
    glmFacetNormals(originMesh);
    glmVertexNormals(samplingMesh, 90.0f);
    glmVertexNormals(originMesh, 90.0f);

    cout << "Origin Mesh Vertices Num: " << originMesh->numvertices << endl;
    cout << "Sampling Mesh Vertices Num: " << samplingMesh->numvertices << endl;
    cout << "--------------------------------------------------" << endl;

    deformationGraph.Init(originMesh, samplingMesh);
}

void reshape(int width, int height)
{
    int base = min(width, height);

    tbReshape(width, height);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLdouble)width / (GLdouble)height, 1.0f, 128.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.5f);
}
vector<vector<vector3>> GetBoundingBox(vector3 left_top, vector3 right_bottom)
{
    vector<vector<vector3>> res;
    vector<vector3> polygon;

    vector3 center = (right_bottom - left_top) / 2 + left_top;
    vector3 up = vector3(0, left_top[1] - center[1], 0);
    vector3 right = vector3(right_bottom[0] - center[0], 0, 0);
    vector3 front = vector3(0, 0, left_top[2] - center[2]);
    // front
    polygon.clear();
    polygon.push_back(vector3(center + front - up - right));
    polygon.push_back(vector3(center + front - up + right));
    polygon.push_back(vector3(center + front + up + right));
    polygon.push_back(vector3(center + front + up - right));
    polygon.push_back(vector3(0, 0, 1));
    res.push_back(polygon);
    // back
    polygon.clear();
    polygon.push_back(vector3(center - front - up - right));
    polygon.push_back(vector3(center - front - up + right));
    polygon.push_back(vector3(center - front + up + right));
    polygon.push_back(vector3(center - front + up - right));
    polygon.push_back(vector3(0, 0, -1));
    res.push_back(polygon);
    // top
    polygon.clear();
    polygon.push_back(vector3(center + front + up - right));
    polygon.push_back(vector3(center + front + up + right));
    polygon.push_back(vector3(center - front + up + right));
    polygon.push_back(vector3(center - front + up - right));
    polygon.push_back(vector3(0, 1, 0));
    res.push_back(polygon);
    // bottom
    polygon.clear();
    polygon.push_back(vector3(center + front - up - right));
    polygon.push_back(vector3(center + front - up + right));
    polygon.push_back(vector3(center - front - up + right));
    polygon.push_back(vector3(center - front - up - right));
    polygon.push_back(vector3(0, -1, 0));
    res.push_back(polygon);
    // left
    polygon.clear();
    polygon.push_back(vector3(center + front + up - right));
    polygon.push_back(vector3(center + front - up - right));
    polygon.push_back(vector3(center - front - up - right));
    polygon.push_back(vector3(center - front + up - right));
    polygon.push_back(vector3(1, 0, 0));
    res.push_back(polygon);
    // right
    polygon.clear();
    polygon.push_back(vector3(center + front + up + right));
    polygon.push_back(vector3(center + front - up + right));
    polygon.push_back(vector3(center - front - up + right));
    polygon.push_back(vector3(center - front + up + right));
    polygon.push_back(vector3(-1, 0, 0));
    res.push_back(polygon);

    return res;
}

void RenderControlBox(vector3 translate)
{
    glPushMatrix();

    tbMatrix();

    glTranslatef(translate.x, translate.y, translate.z);
    glRotatef(180, 0, 1, 0);
    
    for (int handleIter = 0; handleIter < handles.size(); handleIter++)
    {
        vector3 left_top(INT_MIN, INT_MIN, INT_MIN), right_bottom(INT_MAX, INT_MAX, INT_MAX);
        vector<vector<vector3>> box;

        for (int vertIter = 0; vertIter < handles[handleIter].size(); vertIter++)
        {
            int idx = handles[handleIter][vertIter];
            glVertex3fv((float*)&samplingMesh->vertices[3 * idx]);

            left_top[0] = max(left_top[0], samplingMesh->vertices[3 * idx + 0]);
            left_top[1] = max(left_top[1], samplingMesh->vertices[3 * idx + 1]);
            left_top[2] = max(left_top[2], samplingMesh->vertices[3 * idx + 2]);

            right_bottom[0] = min(right_bottom[0], samplingMesh->vertices[3 * idx + 0]);
            right_bottom[1] = min(right_bottom[1], samplingMesh->vertices[3 * idx + 1]);
            right_bottom[2] = min(right_bottom[2], samplingMesh->vertices[3 * idx + 2]);
        }

        box = GetBoundingBox(left_top, right_bottom);
        glColor3fv(colors[handleIter % colors.size()]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
        for (int i = 0; i < box.size(); ++i)
        {
            glBegin(GL_POLYGON);
            glNormal3fv((float*)&box[i][4]);
            for (int j = 0; j < 4; ++j)
            {
                glVertex3fv((float*)&box[i][j]);
            }
            glEnd();
        }
        glDisable(GL_LIGHTING);
    }
    
    glPopMatrix();
}

void RenderCoordinate()
{
    glPushMatrix();

    tbMatrix();

    glLineWidth(5.0f);
    glColor3f(1, 1, 1);
    for (int i = 0; i < 3; ++i)
    {
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f((i == 0) * 1.5, (i == 1) * 1.5, (i == 2) * 1.5);
        glEnd();
    }

    glPopMatrix();
}

void RenderMesh(GLMmodel* model, vector3 translate, vector3 color)
{
    glPushMatrix();

    tbMatrix();

    glTranslatef(translate.x, translate.y, translate.z);
    glRotatef(180, 0, 1, 0);

    glEnable(GL_LIGHTING);
    glColor3f(1.0f, 1.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glmDraw(model, GLM_SMOOTH);

    glPolygonOffset(1.0f, 1.0f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glLineWidth(1.0f);
    glColor3f(color.x, color.y, color.z);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glmDraw(model, GLM_SMOOTH);

    glDisable(GL_LIGHTING);
      
    glPopMatrix();
}

void RenderDeformationGraph(vector3 translate, vector3 color)
{
    glPushMatrix();

    glPopMatrix();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    RenderCoordinate();
    RenderControlBox(vector3(0.0, 0.0, 0.0));
    RenderMesh(originMesh, vector3(1.5, 0.0, 0.0), vector3(0.6, 0.0, 0.0));
    RenderMesh(samplingMesh, vector3(0.0, 0.0, 0.0), vector3(0.0, 0.6, 0.0));
    
    glFlush();
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
    tbMouse(button, state, x, y);

    if (current_mode == SELECT_MODE && button == GLUT_RIGHT_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            select_x = x;
            select_y = y;
        }
        else
        {
            vector<int> this_handle;

            // project all mesh vertices to current viewport
            for (int vertIter = 0; vertIter < samplingMesh->numvertices; vertIter++)
            {
                vector3 pt(samplingMesh->vertices[3 * vertIter + 0], samplingMesh->vertices[3 * vertIter + 1], samplingMesh->vertices[3 * vertIter + 2]);
                vector2 pos = projection_helper(pt);

                // if the projection is inside the box specified by mouse click&drag, add it to current handle
                if (pos.x >= select_x && pos.y >= select_y && pos.x <= x && pos.y <= y)
                {
                    this_handle.push_back(vertIter);
                }
            }
            if (this_handle.size() != 0)
            {
                handles.push_back(this_handle);
                deformationGraph.SetControlPoints(handles);
            }
        }
    }
    else if (current_mode == DEFORM_MODE && button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        // project all handle vertices to current viewport
        // see which is closest to selection point
        double min_dist = 999999;
        int handle_id = -1;
        for (int handleIter = 0; handleIter < handles.size(); handleIter++)
        {
            for (int vertIter = 0; vertIter < handles[handleIter].size(); vertIter++)
            {
                int idx = handles[handleIter][vertIter];
                vector3 pt(samplingMesh->vertices[3 * idx + 0], samplingMesh->vertices[3 * idx + 1], samplingMesh->vertices[3 * idx + 2]);
                vector2 pos = projection_helper(pt);

                double this_dist = sqrt((double)(pos.x - x) * (pos.x - x) + (double)(pos.y - y) * (pos.y - y));
                if (this_dist < min_dist)
                {
                    min_dist = this_dist;
                    handle_id = handleIter;
                }
            }
        }

        selected_handle_id = handle_id;
        deform_mesh_flag = true;
    }

    if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
    {
        deform_mesh_flag = false;
    }
    if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP && current_mode == DEFORM_MODE)
    {
        deformationGraph.Run();
    }

    return;
}

void motion(int x, int y)
{
    tbMotion(x, y);

    // if in deform mode and a handle is selected, deform the mesh
    if (current_mode == DEFORM_MODE && deform_mesh_flag == true)
    {
        matrix44 m;
        vector4 vec = vector4((float)(x - last_x) / 1000.0f, (float)(y - last_y) / 1000.0f, 0.0, 1.0);

        gettbMatrix((float*)&m);
        vec = m * vec;
        
        deformationGraph.SetControlPointsTranslate(selected_handle_id, vector3(vec));
        //deformationGraph.Run();
    }

    last_x = x;
    last_y = y;
}

void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 'd':
        current_mode = DEFORM_MODE;
        cout << "deform mode" << endl;
        break;
    default:
    case 's':
        current_mode = SELECT_MODE;
        cout << "select mode" << endl;
        break;
    }
}

void timf(int value)
{
    glutPostRedisplay();
    glutTimerFunc(1, timf, 0);
}

int main(int argc, char** argv)
{
    int WindWidth = 400;
    int WindHeight = 400;

    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };
    
    float red[] = { 1.0, 0.0, 0.0 };
    colors.push_back(red);
    float yellow[] = { 1.0, 1.0, 0.0 };
    colors.push_back(yellow);
    float blue[] = { 0.0, 1.0, 1.0 };
    colors.push_back(blue);
    float green[] = { 0.0, 1.0, 0.0 };
    colors.push_back(green);

    glutInit(&argc, argv);
    glutInitWindowSize(WindWidth, WindHeight);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow("Embedded Deformation");

    init();

    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
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
    
    glutTimerFunc(40, timf, 0);


    glutMainLoop();

    return 0;
}
