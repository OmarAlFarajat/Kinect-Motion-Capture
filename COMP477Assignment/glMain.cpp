/*	COMP 477 Assignment 2 Problem 4
*	Team 07
*	
*	Some code was adapted from tutorials made available online by Edward Zhang, a PhD student at the University of Washington. 
*	Source: https://homes.cs.washington.edu/~edzhang/tutorials/kinect/kinect4.html
*/

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include "glut.h"
#else
#include <GL/freeglut.h>
#endif
#endif

#include "main.h"
#include <Ole2.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <cstdio>
#include <Windows.h>
#include <iostream>
#include <cmath>
#include <cstring>
#include "skeleton.h"
#include "defMesh.h"
#include "euler.h"

using namespace std;

// OpenGL Variables
long depthToRgbMap[width * height * 2];
// We'll be using buffer objects to store the kinect point cloud
GLuint vboId;
GLuint cboId;

// Kinect variables
HANDLE depthStream;
HANDLE rgbStream;
INuiSensor* sensor;

// Stores the coordinates of each joint
Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];
Matrix4 localBoneOrientation[NUI_SKELETON_POSITION_COUNT];
Matrix4 localBoneOrientationInv[NUI_SKELETON_POSITION_COUNT];
Matrix4 globalBoneOrientation[NUI_SKELETON_POSITION_COUNT];

//Create Mesh
DefMesh myDefMesh;

//Switches
int meshModel = 0;
bool drawSkeleton = true;
bool recording = false;
bool firstKeyframe = true;

float keyframeTime = 0.25f;
float keyframeStart = 0.0f;

//Window parameters
//int width = 1024;
//int height = 768;
///* Ortho (if used) */
double _left = 0.0;		/* ortho view volume params */
double _right = 0.0;
double _bottom = 0.0;
double _top = 0.0;
double _zNear = 0.1;
double _zFar = 50.0;
double fovy = 45.0;
double prev_z = 0;

//key frame control
int interpolation_model = 0;
int key_frame_index = 0;
bool add_new_frame = false;

//Model matrices
double _matrix[16];
double _matrixI[16];

/* Mouse Interface  */
int _mouseX = 0;		/* mouse control variables */
int _mouseY = 0;
bool _mouseLeft = false;
bool _mouseMiddle = false;
bool _mouseRight = false;
bool _edit = false;

double _dragPosX = 0.0;
double _dragPosY = 0.0;
double _dragPosZ = 0.0;

///
/// Kinect functions
///

bool initKinect() {
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,        // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		NULL,     // Event handle
		&depthStream);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,      // Image stream flags, e.g. near mode
		2,      // Number of frames to buffer
		NULL,   // Event handle
		&rgbStream);
	sensor->NuiSkeletonTrackingEnable(NULL, 0); // NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT for only upper body
	return sensor;
}

// Modified: To get bone orientation data from Kinect
void getSkeletalData() {
	NUI_SKELETON_FRAME skeletonFrame = { 0 };
	if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
		sensor->NuiTransformSmooth(&skeletonFrame, NULL);
		// Loop over all sensed skeletons
		for (int z = 0; z < NUI_SKELETON_COUNT; ++z) {
			const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];
			NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];

			bool orientationsCalculated = NuiSkeletonCalculateBoneOrientations(&skeleton, boneOrientations);

			// Check the state of the skeleton
			if (skeleton.eTrackingState == NUI_SKELETON_TRACKED) {

				// Copy the joint positions into our array
				for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
					skeletonPosition[i] = skeleton.SkeletonPositions[i];
					if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED) {
						skeletonPosition[i].w = 0;
					}
				}

				///////// Copy orientations
				localBoneOrientation[NUI_SKELETON_POSITION_HAND_LEFT] = boneOrientations[NUI_SKELETON_POSITION_HAND_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_WRIST_LEFT] = boneOrientations[NUI_SKELETON_POSITION_WRIST_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_ELBOW_LEFT] = boneOrientations[NUI_SKELETON_POSITION_ELBOW_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_SHOULDER_LEFT] = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_HAND_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_HAND_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_WRIST_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_WRIST_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_ELBOW_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_ELBOW_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_SHOULDER_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_HEAD] = boneOrientations[NUI_SKELETON_POSITION_HEAD].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_SHOULDER_CENTER] = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_CENTER].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_SPINE] = boneOrientations[NUI_SKELETON_POSITION_SPINE].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_HIP_CENTER] = boneOrientations[NUI_SKELETON_POSITION_HIP_CENTER].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_HIP_LEFT] = boneOrientations[NUI_SKELETON_POSITION_HIP_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_HIP_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_HIP_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_KNEE_LEFT] = boneOrientations[NUI_SKELETON_POSITION_KNEE_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_KNEE_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_KNEE_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_ANKLE_LEFT] = boneOrientations[NUI_SKELETON_POSITION_ANKLE_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_ANKLE_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_ANKLE_RIGHT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_FOOT_LEFT] = boneOrientations[NUI_SKELETON_POSITION_FOOT_LEFT].hierarchicalRotation.rotationMatrix;
				localBoneOrientation[NUI_SKELETON_POSITION_FOOT_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_FOOT_RIGHT].hierarchicalRotation.rotationMatrix;


				globalBoneOrientation[NUI_SKELETON_POSITION_HAND_LEFT] = boneOrientations[NUI_SKELETON_POSITION_HAND_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_WRIST_LEFT] = boneOrientations[NUI_SKELETON_POSITION_WRIST_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_ELBOW_LEFT] = boneOrientations[NUI_SKELETON_POSITION_ELBOW_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_SHOULDER_LEFT] = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_HAND_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_HAND_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_WRIST_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_WRIST_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_ELBOW_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_ELBOW_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_SHOULDER_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_HEAD] = boneOrientations[NUI_SKELETON_POSITION_HEAD].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_SHOULDER_CENTER] = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_CENTER].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_SPINE] = boneOrientations[NUI_SKELETON_POSITION_SPINE].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_HIP_CENTER] = boneOrientations[NUI_SKELETON_POSITION_HIP_CENTER].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_HIP_LEFT] = boneOrientations[NUI_SKELETON_POSITION_HIP_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_HIP_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_HIP_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_KNEE_LEFT] = boneOrientations[NUI_SKELETON_POSITION_KNEE_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_KNEE_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_KNEE_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_ANKLE_LEFT] = boneOrientations[NUI_SKELETON_POSITION_ANKLE_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_ANKLE_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_ANKLE_RIGHT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_FOOT_LEFT] = boneOrientations[NUI_SKELETON_POSITION_FOOT_LEFT].absoluteRotation.rotationMatrix;
				globalBoneOrientation[NUI_SKELETON_POSITION_FOOT_RIGHT] = boneOrientations[NUI_SKELETON_POSITION_FOOT_RIGHT].absoluteRotation.rotationMatrix;



				return; // Only take the data for one skeleton

			}
		}
	}
}

// Modified: To include more joints from Kinect skeleton
void drawKinectData() {
	getSkeletalData();

	// Draw some arms
	const Vector4& lh = skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT];
	const Vector4& lw = skeletonPosition[NUI_SKELETON_POSITION_WRIST_LEFT];
	const Vector4& le = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_LEFT];
	const Vector4& ls = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_LEFT];
	const Vector4& rh = skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT];
	const Vector4& rw = skeletonPosition[NUI_SKELETON_POSITION_WRIST_RIGHT];
	const Vector4& re = skeletonPosition[NUI_SKELETON_POSITION_ELBOW_RIGHT];
	const Vector4& rs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
	const Vector4& head = skeletonPosition[NUI_SKELETON_POSITION_HEAD];
	const Vector4& cs = skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_CENTER];
	const Vector4& spine = skeletonPosition[NUI_SKELETON_POSITION_SPINE];
	const Vector4& hipc = skeletonPosition[NUI_SKELETON_POSITION_HIP_CENTER];
	const Vector4& hipl = skeletonPosition[NUI_SKELETON_POSITION_HIP_LEFT];
	const Vector4& hipr = skeletonPosition[NUI_SKELETON_POSITION_HIP_RIGHT];
	const Vector4& kl = skeletonPosition[NUI_SKELETON_POSITION_KNEE_LEFT];
	const Vector4& kr = skeletonPosition[NUI_SKELETON_POSITION_KNEE_RIGHT];
	const Vector4& al = skeletonPosition[NUI_SKELETON_POSITION_ANKLE_LEFT];
	const Vector4& ar = skeletonPosition[NUI_SKELETON_POSITION_ANKLE_RIGHT];
	const Vector4& fl = skeletonPosition[NUI_SKELETON_POSITION_FOOT_LEFT];
	const Vector4& fr = skeletonPosition[NUI_SKELETON_POSITION_FOOT_RIGHT];

	glLineWidth(5.0f);
	glColor3f(0.f, 1.f, 0.f);
	glBegin(GL_LINES);


	if (head.w > 0) {
		glVertex3f(lh.x, lh.y, lh.z);
		glVertex3f(lw.x, lw.y, lw.z);

		glVertex3f(lw.x, lw.y, lw.z);
		glVertex3f(le.x, le.y, le.z);

		glVertex3f(le.x, le.y, le.z);
		glVertex3f(ls.x, ls.y, ls.z);

		glVertex3f(ls.x, ls.y, ls.z);
		glVertex3f(cs.x, cs.y, cs.z);

		glVertex3f(rh.x, rh.y, rh.z);
		glVertex3f(rw.x, rw.y, rw.z);

		glVertex3f(rw.x, rw.y, rw.z);
		glVertex3f(re.x, re.y, re.z);

		glVertex3f(re.x, re.y, re.z);
		glVertex3f(rs.x, rs.y, rs.z);

		glVertex3f(rs.x, rs.y, rs.z);
		glVertex3f(cs.x, cs.y, cs.z);

		glVertex3f(head.x, head.y, head.z);
		glVertex3f(cs.x, cs.y, cs.z);

		glVertex3f(cs.x, cs.y, cs.z);
		glVertex3f(spine.x, spine.y, spine.z);

		glVertex3f(spine.x, spine.y, spine.z);
		glVertex3f(hipc.x, hipc.y, hipc.z);

		glVertex3f(hipc.x, hipc.y, hipc.z);
		glVertex3f(hipl.x, hipl.y, hipl.z);

		glVertex3f(hipl.x, hipl.y, hipl.z);
		glVertex3f(kl.x, kl.y, kl.z);

		glVertex3f(kl.x, kl.y, kl.z);
		glVertex3f(al.x, al.y, al.z);

		glVertex3f(al.x, al.y, al.z);
		glVertex3f(fl.x, fl.y, fl.z);

		glVertex3f(hipc.x, hipc.y, hipc.z);
		glVertex3f(hipr.x, hipr.y, hipr.z);

		glVertex3f(hipr.x, hipr.y, hipr.z);
		glVertex3f(kr.x, kr.y, kr.z);

		glVertex3f(kr.x, kr.y, kr.z);
		glVertex3f(ar.x, ar.y, ar.z);

		glVertex3f(ar.x, ar.y, ar.z);
		glVertex3f(fr.x, fr.y, fr.z);
	}

	glEnd();
}

///
/// Dr. Popa's functions
///
double vlen(double x, double y, double z)
{
    return sqrt(x * x + y * y + z * z);
}

void invertMatrix(const GLdouble * m, GLdouble * out)
{

/* NB. OpenGL Matrices are COLUMN major. */
#define MAT(m,r,c) (m)[(c)*4+(r)]

/* Here's some shorthand converting standard (row,column) to index. */
#define m11 MAT(m,0,0)
#define m12 MAT(m,0,1)
#define m13 MAT(m,0,2)
#define m14 MAT(m,0,3)
#define m21 MAT(m,1,0)
#define m22 MAT(m,1,1)
#define m23 MAT(m,1,2)
#define m24 MAT(m,1,3)
#define m31 MAT(m,2,0)
#define m32 MAT(m,2,1)
#define m33 MAT(m,2,2)
#define m34 MAT(m,2,3)
#define m41 MAT(m,3,0)
#define m42 MAT(m,3,1)
#define m43 MAT(m,3,2)
#define m44 MAT(m,3,3)

    GLdouble det;
    GLdouble d12, d13, d23, d24, d34, d41;
    GLdouble tmp[16];		/* Allow out == in. */

    /* Inverse = adjoint / det. (See linear algebra texts.) */

    /* pre-compute 2x2 dets for last two rows when computing */
    /* cofactors of first two rows. */
    d12 = (m31 * m42 - m41 * m32);
    d13 = (m31 * m43 - m41 * m33);
    d23 = (m32 * m43 - m42 * m33);
    d24 = (m32 * m44 - m42 * m34);
    d34 = (m33 * m44 - m43 * m34);
    d41 = (m34 * m41 - m44 * m31);

    tmp[0] = (m22 * d34 - m23 * d24 + m24 * d23);
    tmp[1] = -(m21 * d34 + m23 * d41 + m24 * d13);
    tmp[2] = (m21 * d24 + m22 * d41 + m24 * d12);
    tmp[3] = -(m21 * d23 - m22 * d13 + m23 * d12);

    /* Compute determinant as early as possible using these cofactors. */
    det = m11 * tmp[0] + m12 * tmp[1] + m13 * tmp[2] + m14 * tmp[3];

    /* Run singularity test. */
    if (det == 0.0) {
	/* printf("invert_matrix: Warning: Singular matrix.\n"); */
/* 	  memcpy(out,_identity,16*sizeof(double)); */
    } else {
	GLdouble invDet = 1.0 / det;
	/* Compute rest of inverse. */
	tmp[0] *= invDet;
	tmp[1] *= invDet;
	tmp[2] *= invDet;
	tmp[3] *= invDet;

	tmp[4] = -(m12 * d34 - m13 * d24 + m14 * d23) * invDet;
	tmp[5] = (m11 * d34 + m13 * d41 + m14 * d13) * invDet;
	tmp[6] = -(m11 * d24 + m12 * d41 + m14 * d12) * invDet;
	tmp[7] = (m11 * d23 - m12 * d13 + m13 * d12) * invDet;

	/* Pre-compute 2x2 dets for first two rows when computing */
	/* cofactors of last two rows. */
	d12 = m11 * m22 - m21 * m12;
	d13 = m11 * m23 - m21 * m13;
	d23 = m12 * m23 - m22 * m13;
	d24 = m12 * m24 - m22 * m14;
	d34 = m13 * m24 - m23 * m14;
	d41 = m14 * m21 - m24 * m11;

	tmp[8] = (m42 * d34 - m43 * d24 + m44 * d23) * invDet;
	tmp[9] = -(m41 * d34 + m43 * d41 + m44 * d13) * invDet;
	tmp[10] = (m41 * d24 + m42 * d41 + m44 * d12) * invDet;
	tmp[11] = -(m41 * d23 - m42 * d13 + m43 * d12) * invDet;
	tmp[12] = -(m32 * d34 - m33 * d24 + m34 * d23) * invDet;
	tmp[13] = (m31 * d34 + m33 * d41 + m34 * d13) * invDet;
	tmp[14] = -(m31 * d24 + m32 * d41 + m34 * d12) * invDet;
	tmp[15] = (m31 * d23 - m32 * d13 + m33 * d12) * invDet;

	memcpy(out, tmp, 16 * sizeof(GLdouble));
    }

#undef m11
#undef m12
#undef m13
#undef m14
#undef m21
#undef m22
#undef m23
#undef m24
#undef m31
#undef m32
#undef m33
#undef m34
#undef m41
#undef m42
#undef m43
#undef m44
#undef MAT
}

void matrix_interpolation(int key_frame_index);

void euler_interpolation(int key_frame_index);

void quaternion_lerp(int key_frame_index);

void quaternion_slerp(int key_frame_index);

void animation_controller(int interpolation_mode, int key_frame_index);

void next_frame(int key_frame_index);

void pos(double *px, double *py, double *pz, const int x, const int y,
	 const int *viewport)
{
    /*
       Use the ortho projection and viewport information
       to map from mouse co-ordinates back into world 
       co-ordinates
     */

    *px = (double) (x - viewport[0]) / (double) (viewport[2]);
    *py = (double) (y - viewport[1]) / (double) (viewport[3]);

    *px = _left + (*px) * (_right - _left);
    *py = _top + (*py) * (_bottom - _top);
    *pz = _zNear;
}

void getMatrix()
{
    glGetDoublev(GL_MODELVIEW_MATRIX, _matrix);
    invertMatrix(_matrix, _matrixI);
}

void init()
{
    glMatrixMode(GL_MODELVIEW_MATRIX);

      //Light values and coordinates
     GLfloat ambientLight[] = { 0.3f, 0.3f, 0.3f, 1.0f };
     GLfloat diffuseLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
     GLfloat lightPos[] = {20.0f, 20.0f, 50.0f};
     glEnable(GL_DEPTH_TEST);
     glFrontFace(GL_CCW);
     //glEnable(GL_CULL_FACE);
     glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
     // Hidden surface removal // Counterclockwise polygons face out // Do not calculate inside of jet // Enable lighting
     glEnable(GL_LIGHTING);
     // Set up and enable light 0
     glLightfv(GL_LIGHT0,GL_AMBIENT,ambientLight);
     glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight);
     glEnable(GL_LIGHT0);
     // Enable color tracking
     glEnable(GL_COLOR_MATERIAL);
     // Set material properties to follow glColor values
     glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

     glClearColor(0.2f, 0.2f, 0.2f, 3.0f );
    
     //Rescale normals to unit length
     glEnable(GL_NORMALIZE);
     glLightfv(GL_LIGHT0,GL_POSITION,lightPos);

     glShadeModel(GL_FLAT);
     getMatrix(); //Init matrix

     //Translate camera
     glPushMatrix();
     glLoadIdentity();
     glTranslatef(0,0,-5.0);
     glMultMatrixd(_matrix);
     getMatrix();
     glPopMatrix();

}

void changeSize(int w, int h)
{
    glViewport(0, 0, w, h);


    _top = 1.0;
    _bottom = -1.0;
    _left = -(double) w / (double) h;
    _right = -_left;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    /* glOrtho(_left,_right,_bottom,_top,_zNear,_zFar);  Ortho */
    gluPerspective(fovy, (double) w / (double) h, _zNear, _zFar);	/* PErspective for stereo */

    glMatrixMode(GL_MODELVIEW);
}

void timerFunction(int value)       
{
    glutTimerFunc(10,timerFunction,1);
    glutPostRedisplay();
}

void mouseEvent(int button, int state, int x, int y)
{
    int viewport[4];

    _mouseX = x;
    _mouseY = y;

    if (state == GLUT_UP)
	switch (button) {
    case GLUT_LEFT_BUTTON:
        myDefMesh.mySkeleton.release();
            _mouseLeft =false;
            break;
	case GLUT_MIDDLE_BUTTON:
	    _mouseMiddle = false;
	    break;
	case GLUT_RIGHT_BUTTON:
	    _mouseRight = false;
	    break;
    } else
	switch (button) {
	case GLUT_LEFT_BUTTON:
        myDefMesh.mySkeleton.selectOrReleaseJoint();
        _mouseLeft = true;
        break;
	case GLUT_MIDDLE_BUTTON:
	    _mouseMiddle = true;
	    break;
	case GLUT_RIGHT_BUTTON:
	    _mouseRight = true;
	    break;
    case 4:         //Zoomout
        glLoadIdentity();
        glTranslatef(0,0,-0.1);
        glMultMatrixd(_matrix);
        getMatrix();
        glutPostRedisplay();
        break;
    case 3:         //Zoomin
        glLoadIdentity();
        glTranslatef(0,0,0.1);
        glMultMatrixd(_matrix);
        getMatrix();
        glutPostRedisplay();
        break;
    default:
        break;
        //std::cout<<button<<std::endl;
	}

    glGetIntegerv(GL_VIEWPORT, viewport);
    pos(&_dragPosX, &_dragPosY, &_dragPosZ, x, y, viewport);
}

void mousePassiveFunc(int x, int y)
{
    myDefMesh.mySkeleton.checkHoveringStatus(x, y);
}

void mouseMoveEvent(int x, int y)
{
    if (!myDefMesh.mySkeleton.hasJointSelected)
    {
        bool changed = false;

        const int dx = x - _mouseX;
        const int dy = y - _mouseY;

        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        if (dx == 0 && dy == 0)
            return;

        if (_mouseMiddle || (_mouseLeft && _mouseRight)) {
        /* double s = exp((double)dy*0.01); */
        /* glScalef(s,s,s); */
        /* if(abs(prev_z) <= 1.0) */

        glLoadIdentity();
        glTranslatef(0, 0, dy * 0.01);
        glMultMatrixd(_matrix);

        changed = true;
        } else if (_mouseLeft) {
        double ax, ay, az;
        double bx, by, bz;
        double angle;

        ax = dy;
        ay = dx;
        az = 0.0;
        angle = vlen(ax, ay, az) / (double) (viewport[2] + 1) * 180.0;

        /* Use inverse matrix to determine local axis of rotation */

        bx = _matrixI[0] * ax + _matrixI[4] * ay + _matrixI[8] * az;
        by = _matrixI[1] * ax + _matrixI[5] * ay + _matrixI[9] * az;
        bz = _matrixI[2] * ax + _matrixI[6] * ay + _matrixI[10] * az;

        glRotatef(angle, bx, by, bz);

        changed = true;
        } else if (_mouseRight) {
        double px, py, pz;

        pos(&px, &py, &pz, x, y, viewport);

        glLoadIdentity();
        glTranslatef(px - _dragPosX, py - _dragPosY, pz - _dragPosZ);
        glMultMatrixd(_matrix);

        _dragPosX = px;
        _dragPosY = py;
        _dragPosZ = pz;

        changed = true;
        }

        _mouseX = x;
        _mouseY = y;

        if (changed) {
            getMatrix();
            glutPostRedisplay();
        }
    }
    /*
     * Do joint jobs
     */
    else    
    {
        int jpos[2];
        for(int i=0; i<myDefMesh.mySkeleton.joints.size(); ++i)
        {
            if(myDefMesh.mySkeleton.joints[i].isPicked)
            {
                int pid=myDefMesh.mySkeleton.joints[i].parentID;
                if(pid!=-1)
                {
                    int fviewport[]={0,0,1,1};
                    double projection[16];
                    double modelview[16];
                    
                    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
                    glGetDoublev( GL_PROJECTION_MATRIX, projection );
                    
                    double bl[3];
                    double br[3];
                    double tl[3];
                    
                    gluUnProject(0, 0, 0, modelview, projection, fviewport, &bl[0], &bl[1], &bl[2]);
                    gluUnProject(1, 0, 0, modelview, projection, fviewport, &br[0], &br[1], &br[2]);
                    gluUnProject(0, 1, 0, modelview, projection, fviewport, &tl[0], &tl[1], &tl[2]);
                    
                    double v1[]={br[0]-bl[0], br[1]-bl[1], br[2]-bl[2]};
                    double v2[]={tl[0]-bl[0], tl[1]-bl[1], tl[2]-bl[2]};
                    
                    double cr[]={v1[1]*v2[2]-v1[2]*v2[1], v1[2]*v2[0]-v1[0]*v2[2], v1[0]*v2[1]-v1[1]*v2[0]};
                    double norm=sqrt(cr[0]*cr[0]+cr[1]*cr[1]+cr[2]*cr[2]);
                    
                    cr[0]/=norm;cr[1]/=norm;cr[2]/=norm;
                    
                    jpos[0]=myDefMesh.mySkeleton.joints[pid].screenCoord.x;
                    jpos[1]=myDefMesh.mySkeleton.joints[pid].screenCoord.y;
                    
                    float sc1[3];
                    sc1[0]=x-jpos[0];
                    sc1[1]=-y+jpos[1];
                    
                    float sc2[3];
                    sc2[0]=_mouseX-jpos[0];
                    sc2[1]=-_mouseY+jpos[1];
                    
                    float nsc1=sqrt(sc1[0]*sc1[0]+sc1[1]*sc1[1]);
                    float nsc2=sqrt(sc2[0]*sc2[0]+sc2[1]*sc2[1]);
                    
                    if(nsc1>0 && nsc2>0)
                    {
                        sc1[0]/=nsc1;
                        sc1[1]/=nsc1;
                        
                        sc2[0]/=nsc2;
                        sc2[1]/=nsc2;
                        
                        float cross=(sc1[1]*sc2[0]-sc1[0]*sc2[1]);
                        float angle=asin(cross);
                        
                        float q[16];
                        float v[]={(float)cr[0], (float)cr[1], (float)cr[2]};
                        
                        axisToMat(v, angle, q);
                        myDefMesh.mySkeleton.addRotation(q);
                        
                        myDefMesh.mySkeleton.updateGlobal();
                        myDefMesh.updateVertices();
                    }
                }
                break;
            }
        }
        
        _mouseX = x;
        _mouseY = y;
    }
}

// Modified: To include a recording key
void handleKeyPress(unsigned char key, int x, int y)
{
	switch (key)
	{
	//space
	case 32:
	{
		if (recording == false) {
			std::cout << "RECORDING\n";
			recording = true;
			handleKeyPress('m', 0, 0);
		}
		else {
			std::cout << "STOPPED RECORDING\n";
			recording = false;
		}
		break;
	}

	case 'r':
	{
		for (unsigned i = 0; i < myDefMesh.mySkeleton.joints.size(); i++)
		{
			myDefMesh.mySkeleton.joints[i].resetJointPosition();
		}
		myDefMesh.updateVertices();
		myDefMesh.mySkeleton.clear_key_frame();
		key_frame_index = 0;
		break;
	}
	case 'l':
	{
		if (!_edit)
		{
			std::cout << "Load animation file.\nType the file name\n";
			string file_name;
			std::cin >> file_name;
			myDefMesh.mySkeleton.loadAnimation(file_name);
		}
		break;
	}
	case 's':
	{
		if (_edit)
		{
			std::string name;
			std::cout << "Save animation file.\nGive the file a name\n";
			std::cin >> name;
			myDefMesh.mySkeleton.save_animation(name);
		}
		break;
	}
	case 'p':
	{
		std::cout << "Show animation!\n";
		animation_controller(interpolation_model, -1);
		break;
	}
	case '=':
	{
		if (_edit)
		{

			if (key_frame_index == myDefMesh.mySkeleton.key_frame.size() - 1 && !add_new_frame)
			{
				std::cout << "Last frame. Keep pressing the button will add a new key frame\n";
				add_new_frame = true;
			}
			else if (key_frame_index < myDefMesh.mySkeleton.key_frame.size() - 1 && !add_new_frame && myDefMesh.mySkeleton.key_frame.size() != 0)
			{
				std::cout << "Next key frame.\n";
				next_frame(key_frame_index);
				key_frame_index++;
			}
			else if (add_new_frame || myDefMesh.mySkeleton.key_frame.size() == 0)
			{
				std::cout << "Add new key frame!\n";
				myDefMesh.mySkeleton.save_key_frame();
				key_frame_index = myDefMesh.mySkeleton.key_frame.size();
				add_new_frame = true;
			}
			/*else if (myDefMesh.mySkeleton.key_frame.size() == 0)
			{
				myDefMesh.mySkeleton.save_key_frame();
				add_new_frame
			}*/
		}
		else
		{
			if (key_frame_index < myDefMesh.mySkeleton.key_frame.size() - 1)
			{
				std::cout << "Next\n";
				animation_controller(interpolation_model, key_frame_index);
				key_frame_index++;
			}
			else if (key_frame_index == myDefMesh.mySkeleton.key_frame.size() - 2)
			{
				std::cout << "Last\n";
				animation_controller(interpolation_model, key_frame_index);
			}
			else
			{
				std::cout << "End, plese go back\n";
				key_frame_index = 0;
			}
		}
		break;
	}
	case '-':
	{
		//previous has problem
		if (_edit)
		{

			if (key_frame_index <= myDefMesh.mySkeleton.key_frame.size() && key_frame_index > 0)
			{
				std::cout << "Previous key frame.\n";

				next_frame(key_frame_index);
				key_frame_index--;
				add_new_frame = false;
			}
			else if (key_frame_index == 0)
			{
				std::cout << "First frame\n";
				next_frame(key_frame_index);
			}
		}
		else
		{
			if (key_frame_index < myDefMesh.mySkeleton.key_frame.size() && key_frame_index > 0)
			{
				std::cout << "Previous\n";
				animation_controller(interpolation_model, key_frame_index);
				key_frame_index--;
			}
			else if (key_frame_index == 0)
			{
				std::cout << "First frame\n";
			}
		}
		break;
	}
	case 'j':
	{
		if (!_edit)
		{
			std::cout << "Speed up.\n";
			myDefMesh.interval += 0.01;
		}
		break;
	}
	case 'k':
	{
		if (!_edit)
		{
			std::cout << "Low down.\n";
			myDefMesh.interval -= 0.01;
			myDefMesh.interval <= 0 ? myDefMesh.interval = 0.01 : myDefMesh.interval = myDefMesh.interval;
		}
		break;
	}
	case '1':
	{
		interpolation_model = 1;
		if (!_edit)
		{
			std::cout << "Matrix Interoplation\n";
			//matrix_interpolation();
		}
		break;
	}
	case '2':
	{
		interpolation_model = 2;
		if (!_edit)
		{
			std::cout << "euler interpolation\n";
			//euler_interpolation();
		}
		break;
	}
	case '3':
	{
		interpolation_model = 3;
		if (!_edit)
		{
			std::cout << "quaternion lerp\n";
			//quaternion_lerp();
		}
		break;
	}
	case '4':
	{
		interpolation_model = 4;
		if (!_edit)
		{
			std::cout << "quaternion slerp\n";
			//quaternion_slerp();
		}
		break;
	}
	case 'm':
	{
		_edit = !_edit;
		if (_edit)
		{
			std::cout << "Key frame editing mode.\n";
			key_frame_index = 0;
			add_new_frame = false;
		}
		else
		{
			std::cout << "Animation mode.\n";
			key_frame_index = 0;
			add_new_frame = false;
		}
		/*meshModel = (meshModel+1)%3;*/ break;
	}

	case 'q':
		exit(0);

	}
}

// Added: Updates the mesh's bone orientations to match those of the Kinect skeleton on every call.
void linkMeshToKinect(int mesh, int kinect) {

	myDefMesh.mySkeleton.joints[mesh].isHovered = true;
	myDefMesh.mySkeleton.joints[mesh].isPicked = true;

	myDefMesh.mySkeleton.joints[mesh].global_t[0] = globalBoneOrientation[kinect].M11;
	myDefMesh.mySkeleton.joints[mesh].global_t[1] = globalBoneOrientation[kinect].M12;
	myDefMesh.mySkeleton.joints[mesh].global_t[2] = globalBoneOrientation[kinect].M13;
	myDefMesh.mySkeleton.joints[mesh].global_t[3] = globalBoneOrientation[kinect].M14;

	myDefMesh.mySkeleton.joints[mesh].global_t[4] = globalBoneOrientation[kinect].M21;
	myDefMesh.mySkeleton.joints[mesh].global_t[5] = globalBoneOrientation[kinect].M22;
	myDefMesh.mySkeleton.joints[mesh].global_t[6] = globalBoneOrientation[kinect].M23;
	myDefMesh.mySkeleton.joints[mesh].global_t[7] = globalBoneOrientation[kinect].M24;

	myDefMesh.mySkeleton.joints[mesh].global_t[8] = globalBoneOrientation[kinect].M31;
	myDefMesh.mySkeleton.joints[mesh].global_t[9] = globalBoneOrientation[kinect].M32;
	myDefMesh.mySkeleton.joints[mesh].global_t[10] = globalBoneOrientation[kinect].M33;
	myDefMesh.mySkeleton.joints[mesh].global_t[11] = globalBoneOrientation[kinect].M34;

	myDefMesh.mySkeleton.joints[mesh].global_t[12] = globalBoneOrientation[kinect].M41;
	myDefMesh.mySkeleton.joints[mesh].global_t[13] = globalBoneOrientation[kinect].M42;
	myDefMesh.mySkeleton.joints[mesh].global_t[14] = globalBoneOrientation[kinect].M43;
	myDefMesh.mySkeleton.joints[mesh].global_t[15] = globalBoneOrientation[kinect].M44;


	myDefMesh.mySkeleton.joints[mesh].local_t[0] = localBoneOrientation[kinect].M11;
	myDefMesh.mySkeleton.joints[mesh].local_t[1] = localBoneOrientation[kinect].M12;
	myDefMesh.mySkeleton.joints[mesh].local_t[2] = localBoneOrientation[kinect].M13;
	myDefMesh.mySkeleton.joints[mesh].local_t[3] = localBoneOrientation[kinect].M14;

	myDefMesh.mySkeleton.joints[mesh].local_t[4] = localBoneOrientation[kinect].M21;
	myDefMesh.mySkeleton.joints[mesh].local_t[5] = localBoneOrientation[kinect].M22;
	myDefMesh.mySkeleton.joints[mesh].local_t[6] = localBoneOrientation[kinect].M23;
	myDefMesh.mySkeleton.joints[mesh].local_t[7] = localBoneOrientation[kinect].M24;

	myDefMesh.mySkeleton.joints[mesh].local_t[8] = localBoneOrientation[kinect].M31;
	myDefMesh.mySkeleton.joints[mesh].local_t[9] = localBoneOrientation[kinect].M32;
	myDefMesh.mySkeleton.joints[mesh].local_t[10] = localBoneOrientation[kinect].M33;
	myDefMesh.mySkeleton.joints[mesh].local_t[11] = localBoneOrientation[kinect].M34;

	myDefMesh.mySkeleton.joints[mesh].local_t[12] = localBoneOrientation[kinect].M41;
	myDefMesh.mySkeleton.joints[mesh].local_t[13] = localBoneOrientation[kinect].M42;
	myDefMesh.mySkeleton.joints[mesh].local_t[14] = localBoneOrientation[kinect].M43;
	myDefMesh.mySkeleton.joints[mesh].local_t[15] = localBoneOrientation[kinect].M44;

	myDefMesh.mySkeleton.updateGlobal();
	myDefMesh.updateVertices();

	myDefMesh.mySkeleton.joints[mesh].isHovered = false;
	myDefMesh.mySkeleton.joints[mesh].isPicked = false;
}

// Added: Updates the mesh's bone orientations to match those of the Kinect skeleton on every call.
void updateOrientations() {

	if (skeletonPosition[0].w > 0) {
		
		// Limited articulation. Certain joints are not updated due to irregularities

		//linkMeshToKinect(0, NUI_SKELETON_POSITION_SHOULDER_CENTER);
		linkMeshToKinect(1, NUI_SKELETON_POSITION_SPINE);
		//linkMeshToKinect(2, NUI_SKELETON_POSITION_HIP_CENTER);
		linkMeshToKinect(3, NUI_SKELETON_POSITION_HEAD);

		//linkMeshToKinect(4, NUI_SKELETON_POSITION_HIP_RIGHT);
		//linkMeshToKinect(5, NUI_SKELETON_POSITION_KNEE_RIGHT);
		linkMeshToKinect(6, NUI_SKELETON_POSITION_ANKLE_RIGHT);
		linkMeshToKinect(7, NUI_SKELETON_POSITION_FOOT_RIGHT);

		//linkMeshToKinect(8, NUI_SKELETON_POSITION_HIP_LEFT);
		//linkMeshToKinect(9, NUI_SKELETON_POSITION_KNEE_LEFT);
		linkMeshToKinect(10, NUI_SKELETON_POSITION_ANKLE_LEFT);
		linkMeshToKinect(11, NUI_SKELETON_POSITION_FOOT_LEFT);

		//linkMeshToKinect(12, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
		linkMeshToKinect(13, NUI_SKELETON_POSITION_ELBOW_RIGHT);
		linkMeshToKinect(14, NUI_SKELETON_POSITION_WRIST_RIGHT);

		//linkMeshToKinect(15, NUI_SKELETON_POSITION_SHOULDER_LEFT);
		linkMeshToKinect(16, NUI_SKELETON_POSITION_ELBOW_LEFT);
		linkMeshToKinect(17, NUI_SKELETON_POSITION_WRIST_LEFT);
	}
	

}

// Added: Uses existing keyframe functions to record keyframes if bool recording is true
void recordKinectMotion() {
	if (recording) {

		if (firstKeyframe) {
			handleKeyPress('=', 0, 0);
			keyframeStart = glutGet(GLUT_ELAPSED_TIME)/1000.0f;
			firstKeyframe = false; 
		}
		else {
			
			if (glutGet(GLUT_ELAPSED_TIME)/1000.0f - keyframeStart > keyframeTime) {
				handleKeyPress('=', 0, 0);
				cout << "Keyframe time: " << glutGet(GLUT_ELAPSED_TIME)/1000.0f - keyframeStart << endl;
				keyframeStart = glutGet(GLUT_ELAPSED_TIME)/1000.0f;
			}
		}

		
	}
}

// Modified: To include new function calls
void display()
{
	recordKinectMotion();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glMultMatrixd(_matrix);
	updateOrientations();
	drawKinectData();  
	glPushMatrix();
    myDefMesh.glDraw(meshModel); 
    glPopMatrix();
    glutSwapBuffers();
}

void matrix_interpolation(int key_frame_index)
{
	int m = 0;
	if (key_frame_index == -1)
	{

		for (unsigned i = 0; i < myDefMesh.mySkeleton.key_frame.size() - 1; i++)
		{
			while (m * myDefMesh.interval < 1)
			{
				for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame[i].size(); j++)
				{
					Eigen::Matrix4f temp;
					Eigen::Matrix4f temp1 = myDefMesh.mySkeleton.key_frame[i + 1][j];
					Eigen::Matrix4f temp2 = myDefMesh.mySkeleton.key_frame[i][j];
					temp = (temp1 - temp2) * (m * myDefMesh.interval) + temp2;
					for (unsigned k = 0; k < 4; k++)
					{
						myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp(k, 0);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp(k, 1);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp(k, 2);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 3] = temp(k, 3);
					}
				}
				myDefMesh.mySkeleton.updateGlobal();
				myDefMesh.updateVertices();
				display();
				m++;
			}
			m = 0;
		}
	}
	else
	{
		while (m * myDefMesh.interval < 1)
		{
			for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame[key_frame_index].size(); j++)
			{
				Eigen::Matrix4f temp;
				Eigen::Matrix4f temp1 = myDefMesh.mySkeleton.key_frame[key_frame_index + 1][j];
				Eigen::Matrix4f temp2 = myDefMesh.mySkeleton.key_frame[key_frame_index][j];
				temp = (temp1 - temp2) * (m * myDefMesh.interval) + temp2;
				for (unsigned k = 0; k < 4; k++)
				{
					myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp(k, 0);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp(k, 1);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp(k, 2);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 3] = temp(k, 3);
				}
			}

			myDefMesh.mySkeleton.updateGlobal();
			myDefMesh.updateVertices();
			display();
			m++;
		}
	}

}

void euler_interpolation(int key_frame_index)
{
	int m = 0;
	if (key_frame_index == -1)
	{

		for (unsigned i = 0; i < myDefMesh.mySkeleton.key_frame.size() - 1; i++)
		{
			while (m * myDefMesh.interval < 1)
			{
				for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame[i].size(); j++)
				{
					Eigen::Matrix4f temp;
					Eigen::Matrix4f temp1 = myDefMesh.mySkeleton.key_frame[i + 1][j].normalized();
					Eigen::Matrix4f temp2 = myDefMesh.mySkeleton.key_frame[i][j].normalized();
					EulerAngle euler1;
					euler1.matrix_to_euler(temp1);
					EulerAngle euler2;
					euler2.matrix_to_euler(temp2);
					float attitude, bank, heading = 0;
					bank = (euler1.bank - euler2.bank) * (m * myDefMesh.interval) + euler2.bank;
					heading = (euler1.heading - euler2.heading) * (m * myDefMesh.interval) + euler2.heading;
					attitude = (euler1.attitude - euler2.attitude) * (m * myDefMesh.interval) + euler2.attitude;
					EulerAngle euler(heading, bank, attitude);
					euler.euler_to_eigen_matrix(temp);

					for (unsigned k = 0; k < 4; k++)
					{
						myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp(k, 0);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp(k, 1);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp(k, 2);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 3] = temp(k, 3);
					}
				}
				myDefMesh.mySkeleton.updateGlobal();
				myDefMesh.updateVertices();
				display();
				m++;
			}
			m = 0;
		}


	}
	else
	{
		while (m * myDefMesh.interval < 1)
		{

			for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame[key_frame_index].size(); j++)
			{
				Eigen::Matrix4f temp;
				Eigen::Matrix4f temp1 = myDefMesh.mySkeleton.key_frame[key_frame_index + 1][j].normalized();
				Eigen::Matrix4f temp2 = myDefMesh.mySkeleton.key_frame[key_frame_index][j].normalized();
				EulerAngle euler1;
				euler1.matrix_to_euler(temp1);
				EulerAngle euler2;
				euler2.matrix_to_euler(temp2);
				float attitude, bank, heading = 0;
				bank = (euler1.bank - euler2.bank) * (m * myDefMesh.interval) + euler2.bank;
				heading = (euler1.heading - euler2.heading) * (m * myDefMesh.interval) + euler2.heading;
				attitude = (euler1.attitude - euler2.attitude) * (m * myDefMesh.interval) + euler2.attitude;
				EulerAngle euler(heading, bank, attitude);
				euler.euler_to_eigen_matrix(temp);

				for (unsigned k = 0; k < 4; k++)
				{
					myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp(k, 0);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp(k, 1);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp(k, 2);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 3] = temp(k, 3);
				}
			}

			myDefMesh.mySkeleton.updateGlobal();
			myDefMesh.updateVertices();
			display();
			m++;
		}
	}


}

Eigen::Quaternionf quaternion_minus(Eigen::Quaternionf temp1, Eigen::Quaternionf temp2)
{
	Eigen::Quaternionf temp(temp1.w() - temp2.w(), temp1.x() - temp2.x(), temp1.y() - temp2.y(), temp1.z() - temp2.z());
	return temp;
}

Eigen::Quaternionf quaternion_plus(Eigen::Quaternionf temp1, Eigen::Quaternionf temp2)
{
	Eigen::Quaternionf temp(temp1.w() + temp2.w(), temp1.x() + temp2.x(), temp1.y() + temp2.y(), temp1.z() + temp2.z());
	return temp;
}

Eigen::Quaternionf quaternion_times_float(Eigen::Quaternionf temp1, float sclar)
{
	Eigen::Quaternionf temp(temp1.w() * sclar, temp1.x() * sclar, temp1.y() * sclar, temp1.z() * sclar);
	return temp;
}

void quaternion_lerp(int key_frame_index)
{
	int m = 0;
	myDefMesh.mySkeleton.key_frame_quaternion = myDefMesh.mySkeleton.matrix_to_quaternion(myDefMesh.mySkeleton.key_frame);
	if (key_frame_index == -1)
	{
		for (unsigned i = 0; i < myDefMesh.mySkeleton.key_frame_quaternion.size() - 1; i++)
		{
			while (m * myDefMesh.interval < 1)
			{
				for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame_quaternion[i].size(); j++)
				{
					Eigen::Quaternionf temp1 = myDefMesh.mySkeleton.key_frame_quaternion[i + 1][j].normalized();
					Eigen::Quaternionf temp2 = myDefMesh.mySkeleton.key_frame_quaternion[i][j].normalized();
					Eigen::Quaternionf temp = quaternion_plus(quaternion_times_float((quaternion_minus(temp1, temp2)), (m * myDefMesh.interval)), temp2);
					Eigen::Matrix3f temp_m3 = temp.toRotationMatrix();
					for (unsigned k = 0; k < 3; k++)
					{
						myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp_m3(k, 0);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp_m3(k, 1);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp_m3(k, 2);
					}
					myDefMesh.mySkeleton.joints[j].local_t[3] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[7] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[11] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[12] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[13] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[14] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[15] = 1;
				}
				myDefMesh.mySkeleton.updateGlobal();
				myDefMesh.updateVertices();
				display();
				m++;
			}
			m = 0;
		}


	}
	else
	{
		while (m * myDefMesh.interval < 1)
		{

			for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame_quaternion[key_frame_index].size(); j++)
			{
				Eigen::Quaternionf temp1 = myDefMesh.mySkeleton.key_frame_quaternion[key_frame_index + 1][j].normalized();
				Eigen::Quaternionf temp2 = myDefMesh.mySkeleton.key_frame_quaternion[key_frame_index][j].normalized();
				Eigen::Quaternionf temp = quaternion_plus(quaternion_times_float((quaternion_minus(temp1, temp2)), (m * myDefMesh.interval)), temp2);
				Eigen::Matrix3f temp_m3 = temp.toRotationMatrix();
				for (unsigned k = 0; k < 3; k++)
				{
					myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp_m3(k, 0);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp_m3(k, 1);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp_m3(k, 2);
				}
				myDefMesh.mySkeleton.joints[j].local_t[3] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[7] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[11] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[12] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[13] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[14] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[15] = 1;
			}

			myDefMesh.mySkeleton.updateGlobal();
			myDefMesh.updateVertices();
			display();
			m++;
		}
	}

}

void quaternion_slerp(int key_frame_index)
{
	int m = 0;
	myDefMesh.mySkeleton.key_frame_quaternion = myDefMesh.mySkeleton.matrix_to_quaternion(myDefMesh.mySkeleton.key_frame);
	if (key_frame_index == -1)
	{

		for (unsigned i = 0; i < myDefMesh.mySkeleton.key_frame_quaternion.size() - 1; i++)
		{
			while (m * myDefMesh.interval < 1)
			{
				for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame_quaternion[i].size(); j++)
				{
					Eigen::Quaternionf temp1 = myDefMesh.mySkeleton.key_frame_quaternion[i + 1][j].normalized();
					Eigen::Quaternionf temp2 = myDefMesh.mySkeleton.key_frame_quaternion[i][j].normalized();
					Eigen::Quaternionf temp = temp2.slerp(m * myDefMesh.interval, temp1);
					Eigen::Matrix3f temp_m3 = temp.toRotationMatrix();
					for (unsigned k = 0; k < 3; k++)
					{
						myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp_m3(k, 0);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp_m3(k, 1);
						myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp_m3(k, 2);
					}
					myDefMesh.mySkeleton.joints[j].local_t[3] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[7] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[11] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[12] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[13] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[14] = 0;
					myDefMesh.mySkeleton.joints[j].local_t[15] = 1;
				}
				myDefMesh.mySkeleton.updateGlobal();
				myDefMesh.updateVertices();
				display();
				m++;
			}
			m = 0;
		}

	}
	else
	{
		while (m * myDefMesh.interval < 1)
		{

			for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame_quaternion[key_frame_index].size(); j++)
			{
				Eigen::Quaternionf temp1 = myDefMesh.mySkeleton.key_frame_quaternion[key_frame_index + 1][j].normalized();
				Eigen::Quaternionf temp2 = myDefMesh.mySkeleton.key_frame_quaternion[key_frame_index][j].normalized();
				Eigen::Quaternionf temp = temp2.slerp(m * myDefMesh.interval, temp1);
				Eigen::Matrix3f temp_m3 = temp.toRotationMatrix();
				for (unsigned k = 0; k < 3; k++)
				{
					myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp_m3(k, 0);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp_m3(k, 1);
					myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp_m3(k, 2);
				}
				myDefMesh.mySkeleton.joints[j].local_t[3] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[7] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[11] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[12] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[13] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[14] = 0;
				myDefMesh.mySkeleton.joints[j].local_t[15] = 1;
			}

			myDefMesh.mySkeleton.updateGlobal();
			myDefMesh.updateVertices();
			display();
			m++;
		}
	}


}

void animation_controller(int interpolation_mode, int key_frame_index)
{
	switch (interpolation_mode)
	{
	case 1:
		matrix_interpolation(key_frame_index);
		break;
	case 2:
		euler_interpolation(key_frame_index);
		break;
	case 3:
		quaternion_lerp(key_frame_index);
	case 4:
		quaternion_slerp(key_frame_index);
		break;
	default:
		break;
	}
}

void next_frame(int key_frame_index)
{
	for (unsigned j = 0; j < myDefMesh.mySkeleton.key_frame[key_frame_index].size(); j++)
	{
		Eigen::Matrix4f temp = myDefMesh.mySkeleton.key_frame[key_frame_index][j];
		for (unsigned k = 0; k < 4; k++)
		{
			myDefMesh.mySkeleton.joints[j].local_t[k * 4] = temp(k, 0);
			myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 1] = temp(k, 1);
			myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 2] = temp(k, 2);
			myDefMesh.mySkeleton.joints[j].local_t[k * 4 + 3] = temp(k, 3);
		}
	}
	myDefMesh.mySkeleton.updateGlobal();
	myDefMesh.updateVertices();
	display();
}


int main(int argc, char* argv)
{

    glutInit(&argc, &argv);
    //Print contex info
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);	//double buffer
    glutInitWindowSize(1024, 768);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("COMP477");
	glewInit();

	// Set up array buffers
	const int dataSize = width * height * 3 * 4;
	glGenBuffers(1, &vboId);
	glBindBuffer(GL_ARRAY_BUFFER, vboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &cboId);
	glBindBuffer(GL_ARRAY_BUFFER, cboId);
	glBufferData(GL_ARRAY_BUFFER, dataSize, 0, GL_DYNAMIC_DRAW);


    glutDisplayFunc(display);
	glutIdleFunc(display);
    glutReshapeFunc(changeSize);
    glutTimerFunc(10, timerFunction, 1);

    glutMouseFunc(mouseEvent);
    glutMotionFunc(mouseMoveEvent);
    glutKeyboardFunc(handleKeyPress);
    glutPassiveMotionFunc(mousePassiveFunc);
    init();

	if (initKinect()) 
		std::cout << "Kinect initialized.\n";

	glutMainLoop();
	return 0;
}

