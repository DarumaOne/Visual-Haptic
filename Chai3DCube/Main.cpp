//===========================================================================
/*
This file is part of the CHAI 3D visualization and haptics libraries.
Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License("GPL") version 2
as published by the Free Software Foundation.

For using the CHAI 3D libraries with software that can not be combined
with the GNU GPL, and for taking advantage of the additional benefits
of our support services, please contact CHAI 3D about acquiring a
Professional Edition License.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   2.0.0 $Rev: 269 $
*/
//===========================================================================

#define GLEW_STATIC
#include "GL\glew.h"
//---------------------------------------------------------------------------
//#include <tchar.h>
#include <assert.h>
//#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#define OVR_OS_WIN32
#include "GLFW\glfw3.h"
#include "GLFW\glfw3native.h"
//---------------------------------------------------------------------------
//#include "GL\glut.h"
//#include "math\CMaths.h"
#include "chai3d.h"
#include "CODE.h"
//---------------------------------------------------------------------------
#include "OVR.h"
#include "OVR_CAPI_GL.h"
#include "OVR_CAPI.h"
//---------------------------------------------------------------------------
using namespace std;
using namespace OVR;

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
int WINDOW_SIZE_w = 800;
int WINDOW_SIZE_h = 600;

// mouse menu options (right button)
const int STEREO_RENDERING[2] = { CHAI_STEREO_RIGHT, CHAI_STEREO_LEFT };

const bool l_Spin = false;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight* light;

// a little "chai3D" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW = 0;
int displayH = 0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a pointer of the ODE object grasped by the tool
cODEGenericBody* graspObject;

// grasp position is respect to object
cVector3d graspPos;

// is grasp currently active
bool graspActive = false;

// a small line used to display a grasp
cShapeLine* graspLine;

// maximum stiffness to be used with virtual objects in scene
double stiffnessMax;

// status of the main simulation haptics loop
bool simulationRunning = false;

bool normalMode = true;
bool freeCameraMode = false;

// ODE world
cODEWorld* ODEWorld;

// ODE object
cODEGenericBody* ODEBody0;
cODEGenericBody* ODEBody1;
cODEGenericBody* ODEBody2;

cODEGenericBody* ODEGPlane0;
cODEGenericBody* ODEGPlane1;
cODEGenericBody* ODEGPlane2;
cODEGenericBody* ODEGPlane3;
cODEGenericBody* ODEGPlane4;
cODEGenericBody* ODEGPlane5;

// root resource path
string resoureRoot;

// has exited haptics simulation thread
bool simulationFinished = false;

ovrHmd				hmd;						// The handle of the headset
ovrHmdDesc			hmdDesc;
ovrEyeRenderDesc	eyeRenderDesc[2];			// Description of the VR.
ovrRecti			eyeRenderViewPort[2];		// Useful to remember when varying resolution
ovrPosef			eyePoses[2];
ovrGLTexture		eyeTexture[2];
ovrVector3f			eyeOffsets[2];
ovrVector3f			cameraPosition;
GLFWwindow*			window;
Matrix4f			projectionMatrici[2];
GLuint				texture;
GLuint				framebuffer;
GLuint				depthbuffer;
ovrGLConfig			cfg;
double				lastTime;
int					nbFrames;
float				Yaw;				// Horizontal rotation of the player
Vector3f			Pos(0.0f, 1.6f, -5.0f);		// Position of player
int fpsNo = 1;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when a keyboard key is pressed
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	// option 1
	if (key == '1')
		ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));	//enable gravity

	// option 2
	if (key == '1')
		ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));		//disable gravity
}

//---------------------------------------------------------------------------

// function called before exiting the application
void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loop to terminate
	while (!simulationFinished)
		cSleepMs(100);

	// close haptic device
	//tool->stop;

	glDeleteRenderbuffers(1, &depthbuffer);
	glDeleteTextures(1, &texture);
	glDeleteFramebuffers(1, &framebuffer);

	ovrHmd_Destroy(hmd);
	ovr_Shutdown();

	glfwDestroyWindow(window);
	glfwTerminate();
}

//---------------------------------------------------------------------------

void PrintHapticPosition(cVector3d pos)
{
#define SHOW(a) std::cout<< #a << ": " << (a) << std::endl;
	SHOW(pos);
	cout << "X: " << pos.x
		<< " Y: " << pos.y
		<< " Z: " << pos.z << endl;
}

void HapticNormalMode(cGenericObject* object)
{
	if (tool->getUserSwitch(1))
		ODEWorld->setGravity(cVector3d(0.0, 0.0, 0.0));		//Disable gravity
	if (tool->getUserSwitch(2))
		ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));	// Enable gravity

	// read user switch status
	bool userSwitch = tool->getUserSwitch(0);
	// if the tool is currently grasping an object we simply update the interaction grasp force
	// between the tool and the object (modeled by a virtual spring)
	if (graspActive && userSwitch)
	{
		// retrieve latest position and orientation of grasped ODE object in world coordinates
		cMatrix3d globalGraspObjectRot = graspObject->getGlobalRot();
		cVector3d globalGraspObjectPos = graspObject->getGlobalPos();

		// compute the position of the grasp point on object in global coordinates
		cVector3d globalGraspPos = globalGraspObjectPos + cMul(globalGraspObjectRot, graspPos);

		// retrieve the position of the tool in global coordinates
		cVector3d globalToolPos = tool->getProxyGlobalPos();

		// compute the offset between the tool and grasp point on the object
		cVector3d offset = globalToolPos - globalGraspPos;

		// model a spring between both points
		double STIFFNESS = 4;
		cVector3d force = STIFFNESS * offset;

		// apply attraction force (grasp) onto object
		graspObject->addGlobalForceAtGlobalPos(force, globalGraspPos);

		// scale magnitude and apply opposite force to haptic device
		tool->m_lastComputedGlobalForce.add(cMul(-1.0, force));

		// update both end points of the line which is used for display purposes only
		graspLine->m_pointA = globalGraspPos;
		graspLine->m_pointB = globalToolPos;
	}

	// the user is not or no longer currently grasping the object
	else
	{
		// was the user grasping the object at the previous simulation loop
		if (graspActive)
		{
			// we disable grasping
			graspActive = false;

			// we hide the virtual line between the tool and the grasp point
			graspLine->setShowEnabled(false);

			// we enable haptics interaction between the tool and the previously grasped object
			if (graspObject != NULL)
			{
				graspObject->m_imageModel->setHapticEnabled(true, true);
			}
		}

		// the user is touching an object
		if (object != NULL)
		{
			// check if object is attached to an external ODE parent
			cGenericType* externalParent = object->getExternalParent();
			cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(externalParent);
			if (ODEobject != NULL)
			{
				// get position of tool
				cVector3d pos = tool->m_proxyPointForceModel->m_contactPoint0->m_globalPos;

				// check if user has enabled the user switch to gras the object
				if (userSwitch)
				{
					// a new object is being grasped
					graspObject = ODEobject;

					// retrieve the grasp position on the object in local coordinates
					graspPos = tool->m_proxyPointForceModel->m_contactPoint0->m_localPos;

					// grasp in now active!
					graspActive = true;

					// enable small line which display the offset between the tool and the grasp point
					graspLine->setShowEnabled(true);

					// disable haptic interaction between the tool and the grasped device.
					// this is performed for stability reasons.
					graspObject->m_imageModel->setHapticEnabled(false, true);
				}

				// retrieve the haptic interaction force being applied to the tool
				cVector3d force = tool->m_lastComputedGlobalForce;

				// apply haptic force to ODE object
				cVector3d tmpfrc = cNegate(force);
				ODEobject->addGlobalForceAtGlobalPos(tmpfrc, pos);
			}
		}
	}
}

void HapticCameraMode(void)
{
	if (tool->getUserSwitch(1))
		Yaw += 0.0002f;
	if (tool->getUserSwitch(2))
		Yaw -= 0.0002f;

	freeCameraMode = tool->getUserSwitch(0);

	// hide the proxy device sphere if it's entering the free camera mode
	if (freeCameraMode)
		tool->m_proxySphere->setShowEnabled(false);
	else
		tool->m_proxySphere->setShowEnabled(true);
}

// main haptics loop
void updateHaptics(void)
{
	// simulation clock
	cPrecisionClock simClock;
	simClock.start(true);

	// main haptic simulation loop
	while (!glfwWindowShouldClose(window))
	{
		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		// update position and orientation of tool
		tool->updatePose();

		// compute interaction forces
		tool->computeInteractionForces();

		// check if the tool is touching an object
		cGenericObject* object = tool->m_proxyPointForceModel->m_contactPoint0->m_object;

		if (tool->getUserSwitch(3))
			normalMode = !normalMode;

		if (normalMode)
			HapticNormalMode(object);
		else
			HapticCameraMode();

		// send forces to device
		tool->applyForces();

		// retrieve simulation time and compute next interval
		double time = simClock.getCurrentTimeSeconds();
		double nextSimInterval = cClamp(time, 0.00001, 0.001);

		// reset clock
		simClock.reset();
		simClock.start();

		// update simulation
		ODEWorld->updateDynamics(nextSimInterval);
	}	

	// exit haptics thread
	simulationFinished = true;
}

//---------------------------------------------------------------------------

void BeginFrame(int frameIndex)
{
	ovrHmd_BeginFrame(hmd, frameIndex);

	int width, height;
	float ratio;

	// Get eye poses for both the left and the right eye. g_EyePoses contains all Rift information: orientation, 
	// positional tracking and the IPD in the form of the input variable g_EyeOffsets.
	ovrHmd_GetEyePoses(hmd, frameIndex, eyeOffsets, eyePoses, NULL);

	// Bind the FBO...
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	// Clear...
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glfwGetFramebufferSize(window, &width, &height);
	ratio = width / (float)height;
}

void HapticFreeCameraMode(ovrEyeType eye)
{
	cVector3d deviceGlobalPos = tool->getDeviceGlobalPos();
	cVector3d deviceLocalPos = tool->getDeviceLocalPos();
	cVector3d proxyGlobalPos = tool->getProxyGlobalPos();
	eyePoses[eye].Position.y += deviceGlobalPos.x;
	eyePoses[eye].Position.x -= deviceGlobalPos.y;
	eyePoses[eye].Position.z -= deviceGlobalPos.x;
}

void DrawScene()
{
	for (int eyeIndex = 0; eyeIndex < ovrEye_Count; eyeIndex++)
	{
		ovrEyeType eye = hmd->EyeRenderOrder[eyeIndex];
		eyePoses[eye].Position.x += Yaw;

		glViewport(eyeTexture[eye].OGL.Header.RenderViewport.Pos.x, eyeTexture[eye].OGL.Header.RenderViewport.Pos.y,
			eyeTexture[eye].OGL.Header.RenderViewport.Size.w, eyeTexture[eye].OGL.Header.RenderViewport.Size.h);

		if (freeCameraMode)
			HapticFreeCameraMode(eye);

		camera->renderView(eyeTexture[eye].OGL.Header.RenderViewport.Size.w, eyeTexture[eye].OGL.Header.RenderViewport.Size.h,
			projectionMatrici[eye], eyePoses[eye], cameraPosition, STEREO_RENDERING[eye]);
	}
}

void EndFrame(int &frameIndex)
{
	//glfwSwapBuffers(window);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	ovrHmd_EndFrame(hmd, eyePoses, &eyeTexture[0].Texture);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // Avoid OpenGL state leak in ovrHmd_EndFrame...
	glBindBuffer(GL_ARRAY_BUFFER, 0); // Avoid OpenGL state leak in ovrHmd_EndFrame...

	frameIndex++;
	glfwPollEvents();
}

void CalculateFPS()
{
	double currentTime = glfwGetTime();
	nbFrames++;
	if (currentTime - lastTime >= 1)
	{
		cout << fpsNo << ": " << nbFrames << endl;
		cout << 1000.0 / double(nbFrames) << " ms/frame" << endl;
		nbFrames = 0;
		lastTime += 1.0;
		fpsNo++;
	}
}

// main graphics callback
void updateGraphics(void)
{
	ovrHSWDisplayState hswDisplayState;
	ovrHmd_GetHSWDisplayState(hmd, &hswDisplayState);
	if (hswDisplayState.Displayed)
		ovrHmd_DismissHSWDisplay(hmd);

	// create a thread which starts the main haptics rendering loop
	cThread* hapticsThread = new cThread();
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	int frameIndex = 0;

	while (!glfwWindowShouldClose(window))
	{
		CalculateFPS();
		BeginFrame(frameIndex);
		DrawScene();
		EndFrame(frameIndex);
	}
}

//---------------------------------------------------------------------------

// creates a cube mesh
void createCube(cMesh* a_mesh, double a_size)
{
	const double HALFSIZE = a_size / 2.0;
	int vertices[6][6];

	// face -x
	vertices[0][0] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, -HALFSIZE);
	vertices[0][1] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
	vertices[0][2] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, HALFSIZE);
	vertices[0][3] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, HALFSIZE);

	// face +x
	vertices[1][0] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, -HALFSIZE);
	vertices[1][1] = a_mesh->newVertex(HALFSIZE, HALFSIZE, -HALFSIZE);
	vertices[1][2] = a_mesh->newVertex(HALFSIZE, HALFSIZE, HALFSIZE);
	vertices[1][3] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, HALFSIZE);

	// face -y
	vertices[2][0] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
	vertices[2][1] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, -HALFSIZE);
	vertices[2][2] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, HALFSIZE);
	vertices[2][3] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, HALFSIZE);

	// face +y
	vertices[3][0] = a_mesh->newVertex(HALFSIZE, HALFSIZE, -HALFSIZE);
	vertices[3][1] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, -HALFSIZE);
	vertices[3][2] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, HALFSIZE);
	vertices[3][3] = a_mesh->newVertex(HALFSIZE, HALFSIZE, HALFSIZE);

	// face -z
	vertices[4][0] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, -HALFSIZE);
	vertices[4][1] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, -HALFSIZE);
	vertices[4][2] = a_mesh->newVertex(HALFSIZE, HALFSIZE, -HALFSIZE);
	vertices[4][3] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, -HALFSIZE);

	// face +z
	vertices[5][0] = a_mesh->newVertex(HALFSIZE, -HALFSIZE, HALFSIZE);
	vertices[5][1] = a_mesh->newVertex(HALFSIZE, HALFSIZE, HALFSIZE);
	vertices[5][2] = a_mesh->newVertex(-HALFSIZE, HALFSIZE, HALFSIZE);
	vertices[5][3] = a_mesh->newVertex(-HALFSIZE, -HALFSIZE, HALFSIZE);

	// create triangles
	for (int i = 0; i<6; i++)
	{
		a_mesh->newTriangle(vertices[i][0], vertices[i][1], vertices[i][2]);
		a_mesh->newTriangle(vertices[i][0], vertices[i][2], vertices[i][3]);
	}

	// set material properties to light gray
	a_mesh->m_material.m_ambient.set(0.5f, 0.5f, 0.5f, 1.0f);
	a_mesh->m_material.m_diffuse.set(0.7f, 0.7f, 0.7f, 1.0f);
	a_mesh->m_material.m_specular.set(1.0f, 1.0f, 1.0f, 1.0f);
	a_mesh->m_material.m_emission.set(0.0f, 0.0f, 0.0f, 1.0f);

	// compute normals
	a_mesh->computeAllNormals();

	// compute collision detection algorithm
	a_mesh->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
}

//---------------------------------------------------------------------------

void CreateCamera(cWorld* world)
{
	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and oriente the camera
	camera->set(cVector3d(3.0, 0.0, 0.3),	// camera position (eye)
		cVector3d(0.0, 0.0, 0.0),			// lookat position (target)
		cVector3d(0.0, 0.0, 1.0));			// direction of the "up" vector

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo eye separation and focal length
	float ipd = eyeRenderDesc[ovrEye_Left].HmdToEyeViewOffset.x * -1.0;
	camera->setStereoEyeSeparation(ipd);
	camera->setStereoFocalLength(3.0);
}

void CreateLightSource(cWorld* world)
{
	// create a light source and attach it to the camera
	light = new cLight(world);
	camera->addChild(light);	// attach light to camera
	light->setEnabled(true);	// enable light source
	light->setPos(cVector3d(2.0, 0.5, 1.0));	// position the light source
	light->setDir(cVector3d(-2.0, 0.5, 1.0));	// define the direction of the light beam
	light->m_ambient.set(0.6, 0.6, 0.6);
	light->m_diffuse.set(0.8, 0.8, 0.8);
	light->m_specular.set(0.8, 0.8, 0.8);
}

void CreateWidgets()
{
	//-----------------------------------------------------------------------
	// 2D - WIDGETS
	//-----------------------------------------------------------------------

	// create a 2D bitmap logo
	logo = new cBitmap();

	// add logo to the front plane
	camera->m_front_2Dscene.addChild(logo);

	// load a "chai3d" bitmap image file
	bool fileLoad;
	fileLoad = logo->m_image.loadFromFile("bin/resources/images/chai3d.bmp");

	// position the logo at the bottom left of the screen (pixel coordinates)
	logo->setPos(10, 10, 0);

	// scale the logo along its horizontal and vertical axis
	logo->setZoomHV(0.25, 0.25);

	// here we replace all black pixels (0,0,0) of the logo bitmap
	// with transparent black pixels (0,0,0,0). This allows us to make
	// the background of the logo look transparent.
	logo->m_image.replace(
		cColorb(0, 0, 0),		// original RGB color
		cColorb(0, 0, 0, 0));	// new RGBA color

	// enable transparency
	logo->enableTransparency(true);
}

void CreateHaptics(cWorld* world)
{
	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	cGenericHapticDevice* hapticDevice;
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo info;
	if (hapticDevice)
	{
		info = hapticDevice->getSpecifications();
	}

	// create a 3D tool and add it to the world
	tool = new cGeneric3dofPointer(world);
	world->addChild(tool);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// initialize tool by connecting to haptic device
	tool->start();

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.3);

	// define a radius for the tool (graphical display)
	tool->setRadius(0.05);

	// hide the device sphere. only show proxy.
	tool->m_deviceSphere->setShowEnabled(false);

	// set the physical readius of the proxy.
	proxyRadius = 0.05;
	tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);
	tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->m_proxyPointForceModel->m_useDynamicProxy = true;

	// ajust the color of the tool
	tool->m_materialProxy = tool->m_materialProxyButtonPressed;

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// define a maximum stiffness that can be handled by the current
	// haptic device. The value is scaled to take into account the
	// workspace scale factor
	stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;

	// create a small white line that will be enabled every time the operator
	// grasps an object. The line indicated the connection between the
	// position of the tool and the grasp position on the object
	graspLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	world->addChild(graspLine);
	graspLine->m_ColorPointA.set(1.0, 1.0, 1.0);
	graspLine->m_ColorPointB.set(1.0, 1.0, 1.0);
	graspLine->setShowEnabled(false);
}

void ComposeVirtualScene(cWorld* world, cMesh* &object0, cMesh* &object1, cMesh* &object2)
{
	//-----------------------------------------------------------------------
	// COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	// create an ODE world to simulate dynamic bodies
	ODEWorld = new cODEWorld(world);

	// add ODE world as a node inside world
	world->addChild(ODEWorld);

	// set some gravity
	ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));

	// we create 6 static walls to contains the 3 cubes within a limited workspace
	ODEGPlane0 = new cODEGenericBody(ODEWorld);
	ODEGPlane1 = new cODEGenericBody(ODEWorld);
	ODEGPlane2 = new cODEGenericBody(ODEWorld);
	ODEGPlane3 = new cODEGenericBody(ODEWorld);
	ODEGPlane4 = new cODEGenericBody(ODEWorld);
	ODEGPlane5 = new cODEGenericBody(ODEWorld);

	// create a new ODE object that is automatically added to the ODE world
	ODEBody0 = new cODEGenericBody(ODEWorld);
	ODEBody1 = new cODEGenericBody(ODEWorld);
	ODEBody2 = new cODEGenericBody(ODEWorld);

	// create a cube mesh
	double boxSize = 0.4;
	createCube(object0, boxSize);
	createCube(object1, boxSize);
	createCube(object2, boxSize);

	// define some material properties for each cube
	cMaterial mat0, mat1, mat2;
	mat0.m_ambient.set(0.8, 0.1, 0.4);
	mat0.m_diffuse.set(1.0, 0.15, 0.5);
	mat0.m_specular.set(1.0, 0.2, 0.8);
	mat0.setStiffness(0.5*stiffnessMax);
	mat0.setDynamicFriction(0.8);
	mat0.setStaticFriction(0.8);
	object0->setMaterial(mat0);

	mat1.m_ambient.set(0.2, 0.6, 0.0);
	mat1.m_diffuse.set(0.2, 0.8, 0.0);
	mat1.m_specular.set(0.2, 1.0, 0.0);
	mat1.setStiffness(0.5*stiffnessMax);
	mat1.setDynamicFriction(0.8);
	mat1.setStaticFriction(0.8);
	object1->setMaterial(mat1);

	mat2.m_ambient.set(0.0, 0.2, 0.6);
	mat2.m_diffuse.set(0.0, 0.2, 0.8);
	mat2.m_specular.set(0.0, 0.2, 1.0);
	mat2.setStiffness(0.5*stiffnessMax);
	mat2.setDynamicFriction(0.8);
	mat2.setStaticFriction(0.8);
	object2->setMaterial(mat2);

	// add mesh to ODE object
	ODEBody0->setImageModel(object0);
	ODEBody1->setImageModel(object1);
	ODEBody2->setImageModel(object2);

	// create a dynamic model of the ODE object. Here we decide to use a box just l ike
	// the object mesh we just defined
	ODEBody0->createDynamicBox(boxSize, boxSize, boxSize);
	ODEBody1->createDynamicBox(boxSize, boxSize, boxSize, false, cVector3d(1, 1, 1));
	ODEBody2->createDynamicBox(boxSize, boxSize, boxSize);

	// define some mass properties for each cube
	ODEBody0->setMass(0.05);
	ODEBody1->setMass(0.05);
	ODEBody2->setMass(0.05);

	// set position of each cube
	cVector3d tmpvct;
	tmpvct = cVector3d(0.0, -0.6, -0.5);
	ODEBody0->setPosition(tmpvct);
	tmpvct = cVector3d(0.0, 0.6, -0.5);
	ODEBody1->setPosition(tmpvct);
	tmpvct = cVector3d(0.0, 0.0, -0.5);
	ODEBody2->setPosition(tmpvct);

	// rotate centra cube of 45 degrees (just to show how this works!)
	cMatrix3d rot;
	rot.identity();
	rot.rotate(cVector3d(0, 0, 1), cDegToRad(45));
	ODEBody0->setRotation(rot);

	double size = 1.0;
	ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, 2.0*size), cVector3d(0.0, 0.0, -1.0));
	ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -size), cVector3d(0.0, 0.0, 1.0));
	ODEGPlane2->createStaticPlane(cVector3d(0.0, size, 0.0), cVector3d(0.0, -1.0, 0.0));
	ODEGPlane3->createStaticPlane(cVector3d(0.0, -size, 0.0), cVector3d(0.0, 1.0, 0.0));
	ODEGPlane4->createStaticPlane(cVector3d(size, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0));
	ODEGPlane5->createStaticPlane(cVector3d(-0.8 * size, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));
}

void CreateReflection(cWorld* world, cMesh* &object0, cMesh* &object1, cMesh* &object2)
{
	//////////////////////////////////////////////////////////////////////////
	// Create some reflexion
	//////////////////////////////////////////////////////////////////////////

	// we create an intermediate node to which we will attach
	// a copy of the object located inside the world
	cGenericObject* reflexion = new cGenericObject();

	// set this object as a ghost node so that no haptic interactions or
	// collision detecting take place within the child nodes added to the
	// reflexion node.
	reflexion->setAsGhost(true);

	// add reflexion node to world
	world->addChild(reflexion);

	// turn off culling on each object (objects now get rendered on both sides)
	object0->setUseCulling(false, true);
	object1->setUseCulling(false, true);
	object2->setUseCulling(false, true);

	// create a symmetry rotation matrix (z-plane)
	cMatrix3d rotReflexion;
	rotReflexion.set(1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, -1.0);
	reflexion->setRot(rotReflexion);
	reflexion->setPos(0.0, 0.0, -2.005);

	// add objects to the world
	reflexion->addChild(ODEWorld);
	//reflexion->addChild(tool);
}

void CreateGround(cWorld* world)
{
	//////////////////////////////////////////////////////////////////////////
	// Create a Ground
	//////////////////////////////////////////////////////////////////////////

	// create mesh to model ground surface
	cMesh* ground = new cMesh(world);
	world->addChild(ground);

	// create 4 vertices (one at each corner)
	double groundSize = 2.0;
	int vertices0 = ground->newVertex(-groundSize, -groundSize, 0.0);
	int vertices1 = ground->newVertex(groundSize, -groundSize, 0.0);
	int vertices2 = ground->newVertex(groundSize, groundSize, 0.0);
	int vertices3 = ground->newVertex(-groundSize, groundSize, 0.0);

	// compose surface with 2 triangles
	ground->newTriangle(vertices0, vertices1, vertices2);
	ground->newTriangle(vertices0, vertices2, vertices3);

	// compute surface normals
	ground->computeAllNormals();

	// position ground at the right level
	ground->setPos(0.0, 0.0, -1.0);

	// define some material properties and apply to mesh
	cMaterial matGround;
	matGround.setStiffness(stiffnessMax);
	matGround.setDynamicFriction(0.7);
	matGround.setStaticFriction(1.0);
	matGround.m_ambient.set(0.0, 0.0, 0.0);
	matGround.m_diffuse.set(0.0, 0.0, 0.0);
	matGround.m_specular.set(0.0, 0.0, 0.0);
	ground->setMaterial(matGround);

	// enable and set transparency level of ground
	ground->setTransparencyLevel(0.7);
	ground->setUseTransparency(true);
}

// creates the world
cWorld* theWorld()
{
	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	cWorld* world = new cWorld();

	// set the background colour of the environment
	// the colour is defined by its (R,G,B) components.
	world->setBackgroundColor(0.0, 0.0, 0.0);

	CreateCamera(world);

	CreateLightSource(world);

	CreateWidgets();

	CreateHaptics(world);

	// create a virtual mesh that will be used for the geometry
	// representation of the dynamic body
	cMesh* object0 = new cMesh(world);
	cMesh* object1 = new cMesh(world);
	cMesh* object2 = new cMesh(world);

	ComposeVirtualScene(world, object0, object1, object2);

	CreateReflection(world, object0, object1, object2);

	CreateGround(world);

	return world;
}

// set window
void SetWindow(Sizei resolution, GLFWmonitor* monitor)
{
	window = glfwCreateWindow(resolution.w, resolution.h, "tes", monitor, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	ovrHmd_AttachToWindow(hmd, glfwGetWin32Window(window), NULL, NULL);
	glfwMakeContextCurrent(window);
}

void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

void SetTexture(Sizei renderTargetSize)
{
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, renderTargetSize.w, renderTargetSize.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void SetDepthbuffer(Sizei renderTargetSize)
{
	glGenRenderbuffers(1, &depthbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, depthbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, renderTargetSize.w, renderTargetSize.h);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthbuffer);
}

static void WindowSizeCallback(GLFWwindow* window, int width, int height)
{
	if (width>0 && height>0)
	{
		cfg.OGL.Header.BackBufferSize.w = width;
		cfg.OGL.Header.BackBufferSize.h = height;

		ovrBool configureResult = ovrHmd_ConfigureRendering(hmd, &cfg.Config, ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette |
			ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive, hmd->MaxEyeFov, eyeRenderDesc);
		glUseProgram(0); // Avoid OpenGL state leak in ovrHmd_ConfigureRendering...
		if (!configureResult)
		{
			printf("Configure failed.\n");
			exit(EXIT_FAILURE);
		}
	}
}

Sizei SetResolution(GLFWmonitor* &monitor)
{
	Sizei resolution;

	// Check to see if the hmd in "Direct" or "Extended" mode
	bool directMode = ((hmd->HmdCaps & ovrHmdCap_ExtendDesktop) == 0);
	if (directMode)
	{
		std::cout << "Running in \"Direct\" mode" << endl;
		monitor = NULL;

		resolution.w = hmd->Resolution.w / 2;
		resolution.h = hmd->Resolution.h / 2;
	}
	else
	{
		std::cout << "Running in \"Extended Desktop\" mode" << endl;
		int countMonitor;
		GLFWmonitor** listMonitors = glfwGetMonitors(&countMonitor);

		switch (countMonitor)
		{
		case 0:
			std::cout << "No monitors found." << endl;
			exit(EXIT_FAILURE);
			break;
		case 1:
			std::cout << "Two monitors expected, found only one, using primary." << endl;
			monitor = glfwGetPrimaryMonitor();
			break;
		case 2:
			std::cout << "Two monitors found, using second monitor." << endl;
			monitor = listMonitors[1];
			break;
		default:
			std::cout << "More than two monitors found, using second monitor." << endl;
			monitor = listMonitors[1];
		}

		resolution.w = hmd->Resolution.w;
		resolution.h = hmd->Resolution.h;
	}

	return resolution;
}

void SetEyeRenderViewport(ovrRecti(&eyeRenderViewport)[2], ovrSizei eyeTextureSizes[2], Sizei renderTargetSize)
{
	eyeRenderViewport[ovrEye_Left].Pos = Vector2i(0, 0);
	eyeRenderViewport[ovrEye_Left].Size = eyeTextureSizes[ovrEye_Left];
	eyeRenderViewport[ovrEye_Right].Pos = Vector2i((renderTargetSize.w + 1) / 2, 0);
	eyeRenderViewport[ovrEye_Right].Size = eyeRenderViewport[0].Size;
}

void SetEyeTexture(ovrSizei eyeTextureSizes[2], Sizei renderTargetSize)
{
	ovrRecti eyeRenderViewport[2];
	SetEyeRenderViewport(eyeRenderViewport, eyeTextureSizes, renderTargetSize);

	eyeTexture[ovrEye_Left].OGL.Header.API = ovrRenderAPI_OpenGL;
	eyeTexture[ovrEye_Left].OGL.Header.TextureSize = renderTargetSize;
	eyeTexture[ovrEye_Left].OGL.Header.RenderViewport = eyeRenderViewport[ovrEye_Left];
	eyeTexture[ovrEye_Left].OGL.TexId = texture;

	eyeTexture[ovrEye_Right] = eyeTexture[ovrEye_Left];
	eyeTexture[ovrEye_Right].OGL.Header.RenderViewport = eyeRenderViewport[ovrEye_Right];
}

void SetConfig(Sizei resolution)
{
	cfg.OGL.Header.API = ovrRenderAPI_OpenGL;
	cfg.OGL.Header.BackBufferSize = Sizei(resolution.w, resolution.h);
	cfg.OGL.Header.Multisample = 1;
	cfg.OGL.Window = glfwGetWin32Window(window);
	cfg.OGL.DC = GetDC(cfg.OGL.Window);
}

static void SetStaticLightPositions(void)
{
	GLfloat l_Light0Position[] = { 3.0f, 4.0f, 2.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, l_Light0Position);

	GLfloat l_Light1Position[] = { -3.0f, -4.0f, 2.0f, 0.0f };
	glLightfv(GL_LIGHT1, GL_POSITION, l_Light1Position);
}

void PrintHeadPosition(ovrTrackingState state)
{
	cout << "Head Position:" << endl;
	cout << "X: " << state.HeadPose.ThePose.Position.x
		<< " Y: " << state.HeadPose.ThePose.Position.y
		<< " Z: " << state.HeadPose.ThePose.Position.z << endl;
}

void CheckFramebuffer()
{
	// Check if everything is OK...
	GLenum l_Check = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
	if (l_Check != GL_FRAMEBUFFER_COMPLETE)
	{
		cout << "There is a problem with the FBO." << endl;
		exit(EXIT_FAILURE);
	}
}

void UnbindBuffer()
{
	// Unbind...
	glBindRenderbuffer(GL_RENDERBUFFER, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void StartSensor()
{
	// Start the sensor which provides the Riftfs pose and motion...
	GLuint supportedSensorCaps = ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position;
	GLuint requiredTrackingCaps = 0;
	ovrBool trackingResult = ovrHmd_ConfigureTracking(hmd, supportedSensorCaps, requiredTrackingCaps);
	if (!trackingResult)
	{
		cout << "Could not start tracking..." << endl;
		exit(EXIT_FAILURE);
	}
}

void InitCamera()
{
	// Initial camera position...
	cameraPosition.x = 0.0f;
	cameraPosition.y = 0.0f;
	cameraPosition.z = -3.0f;
}

void InitGlew()
{
	GLenum glewErr = glewInit();
	if (glewInit() != GLEW_OK)
	{
		cerr << glewGetErrorString(glewErr) << endl;
	}
}

void InitHmd()
{
	//Initialize the Rift
	ovr_Initialize();
	hmd = ovrHmd_Create(0);

	//Start debug mode if the Rift isn't detected
	if (!hmd)
		hmd = ovrHmd_CreateDebug(ovrHmd_DK2);
}

// initialization
void initialization(char*argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	InitHmd();

	if (!glfwInit())
		exit(EXIT_FAILURE);

	GLFWmonitor *monitor;
	Sizei resolution = SetResolution(monitor);

	glfwSetErrorCallback(error_callback);

	SetWindow(resolution, monitor);

	InitGlew();

	ovrSizei eyeTextureSizes[2];
	eyeTextureSizes[ovrEye_Left] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Left, hmd->DefaultEyeFov[ovrEye_Left], 1.0f);
	eyeTextureSizes[ovrEye_Right] = ovrHmd_GetFovTextureSize(hmd, ovrEye_Right, hmd->DefaultEyeFov[ovrEye_Right], 1.0f);

	Sizei renderTargetSize;
	renderTargetSize.w = eyeTextureSizes[ovrEye_Left].w + eyeTextureSizes[ovrEye_Right].w;
	renderTargetSize.h = (eyeTextureSizes[ovrEye_Left].h > eyeTextureSizes[ovrEye_Right].h ?
		eyeTextureSizes[ovrEye_Left].h : eyeTextureSizes[ovrEye_Right].h);

	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	// Create Texture
	SetTexture(renderTargetSize);

	// Create Depth Buffer
	SetDepthbuffer(renderTargetSize);

	// Set the texture as our colour attachment #0...
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture, 0);

	// Set the list of draw buffers...
	GLenum drawbuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, drawbuffers); // "1" is the size of DrawBuffers

	CheckFramebuffer();

	UnbindBuffer();

	SetEyeTexture(eyeTextureSizes, renderTargetSize);

	SetConfig(resolution);

	//ovrFovPort eyeFovPorts[2] = { hmd->DefaultEyeFov[0], hmd->DefaultEyeFov[1] };
	ovrHmd_ConfigureRendering(hmd, &cfg.Config, ovrDistortionCap_Chromatic | ovrDistortionCap_Vignette |
		ovrDistortionCap_TimeWarp | ovrDistortionCap_Overdrive, hmd->MaxEyeFov, eyeRenderDesc);
	glUseProgram(0); // Avoid OpenGL state leak in ovrHmd_ConfigureRendering...

	StartSensor();

	// Projection matrici for each eye will not change at runtime, we can set them here...
	projectionMatrici[ovrEye_Left] = ovrMatrix4f_Projection(eyeRenderDesc[ovrEye_Left].Fov, 0.3f, 100.0f, true);
	projectionMatrici[ovrEye_Right] = ovrMatrix4f_Projection(eyeRenderDesc[ovrEye_Right].Fov, 0.3f, 100.0f, true);

	// IPD offset values will not change at runtime, we can set them here...
	eyeOffsets[ovrEye_Left] = eyeRenderDesc[ovrEye_Left].HmdToEyeViewOffset;
	eyeOffsets[ovrEye_Right] = eyeRenderDesc[ovrEye_Right].HmdToEyeViewOffset;

	InitCamera();

	world = theWorld();

	lastTime = glfwGetTime();
	nbFrames = 0;
	Yaw = 0;
}

//===========================================================================
/*
DEMO:    ODE_cubic.cpp

This example illustrates the use of the ODE framework for simulating
haptic interaction with dynamic bodies. In this scene we create 3
cubic meshes that we individually attach to ODE bodies. Haptic interactions
are computer by using the finger-proxy haptic model and forces are
propagated to the ODE representation.
*/
//===========================================================================

int main(int argc, char*argv[])
{
	initialization(argv);

	glfwSetKeyCallback(window, key_callback);
	glfwSetWindowSizeCallback(window, WindowSizeCallback);

	ovrHmd_RecenterPose(hmd);

	// simulation in now running
	simulationRunning = true;

	updateGraphics();

	close();

	// exit
	return (0);
}



