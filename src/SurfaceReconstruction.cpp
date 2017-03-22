/*
Kory Dondzila
Garret Richardson

WarbirdSimulation.cpp
10/05/2016

Main file that initializes the scene, loads model files, scene entities,
performs update and display methods. Takes user input for changing cameras,
update speed, toggle axes, and toggle idle function.

*/

#include "includes/includes.hpp"
#include <string>
#include <vector>

// Initial gl includes required before wglext.h/glxext.h include
#ifdef _WIN32
#include <Windows.h>
#include "includes/wglext.h"
bool WGLExtensionSupported(const char *extension_name)
{
	// this is pointer to function which returns pointer to string with list of all wgl extensions
	PFNWGLGETEXTENSIONSSTRINGEXTPROC _wglGetExtensionsStringEXT = NULL;

	// determine pointer to wglGetExtensionsStringEXT function
	_wglGetExtensionsStringEXT = (PFNWGLGETEXTENSIONSSTRINGEXTPROC)wglGetProcAddress("wglGetExtensionsStringEXT");

	if (strstr(_wglGetExtensionsStringEXT(), extension_name) == NULL)
	{
		// string was not found
		return false;
	}

	// extension is supported
	return true;
}

PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT = NULL;
#endif

#include "Scene.hpp"
#include "DynamicCamera.hpp"
#include "StaticEntity.hpp"

// constants for models:  file names, vertex count, model display size
const int nModels = 1;  // number of models in this scene
char * modelFile[nModels] = { "src/knot.pcd" };
const int nVertices[nModels] = { 1280 };
char * vertexShaderFile = "src/simpleVertex.glsl";
char * fragmentShaderFile = "src/simpleFragment.glsl";


// Shader handles, matrices, etc
GLuint shaderProgram;
GLuint MVP, NormalMatrix, ModelView;  // Model View Projection matrix's handle
GLuint VAO[nModels], buffer[nModels];
GLuint vao;

// model, view, projection matrices and values to create modelMatrix.
glm::mat4 modelMatrix;          // set in display()
glm::mat4 modelViewMatrix;
glm::mat4 projectionMatrix;     // set in reshape()
glm::mat4 ModelViewProjectionMatrix; // set in display();

									 // flags
bool showAxesFlag = false;

// Constants for scene
Scene* scene = Scene::Instance();  // Scene object
int tq = 0, frameCount = 0, updateCount = 0;
double currentTime, lastTime, timeInterval, ulastTime, utimeInterval;
int shipID;

// Constants for cameras
glm::mat4 viewMatrix;  // Current view matrix
StaticCamera* viewingCamera;  // Current camera
int mouseOldX = 0, mouseOldY = 0;
bool rotate = false;
float theta = 0, phi = 0, radius = 2.0f;

// window title string
char titleStr[160];
char baseStr[23] = "Surface Reconstruction";
char pointClousdStr[25] = "  Point Cloud ?";
char verticesStr[25] = "  Vertices ?";
char upsStr[10] = "  U/S ???";
char fpsStr[11] = "  F/S ????";
char viewStr[13] = "  View Front";

// Update window display and projection matrix
void reshape(int width, int height)
{
	projectionMatrix = viewingCamera->UpdateProjectionMatrix(width, height);
	float aspectRatio = (float)width / (float)height;
	glViewport(0, 0, width, height);
	printf("reshape: FOVY = %5.2f, width = %4d height = %4d aspect = %5.2f \n + nearclip = %5f farclip = %5f \n",
		viewingCamera->FOVY(), width, height, aspectRatio, viewingCamera->NearClip(), viewingCamera->FarClip());
}

// Update window title
void updateTitle()
{
	strcpy(titleStr, baseStr);
	strcat(titleStr, pointClousdStr);
	strcat(titleStr, verticesStr);
	strcat(titleStr, upsStr);
	strcat(titleStr, fpsStr);
	strcat(titleStr, viewStr);
	glutSetWindowTitle(titleStr);
}

// Easy to use update functions from GLSL example code, added param
// for shader program
void setUniform(const char *name, float x, GLuint shader = shaderProgram)
{
	GLint loc = glGetUniformLocation(shader, name);
	glUniform1f(loc, x);
}

void setUniform(const char *name, const glm::vec3& v, GLuint shader = shaderProgram)
{
	GLint loc = glGetUniformLocation(shader, name);
	glUniform3fv(loc, 1, glm::value_ptr(v));
}

void setUniform(const char *name, const glm::mat4& m, GLuint shader = shaderProgram)
{
	GLint loc = glGetUniformLocation(shader, name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(m));
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(GL_TRUE);
	glUseProgram(shaderProgram);
	glEnable(GL_POINT_SIZE);
	glPointSize(2.0f);

	// update model matrix
	for (int id : *scene->DrawableObjects())
	{
		StaticEntity* entity = (StaticEntity*)scene->GetEntityFromID(id);
		modelMatrix = entity->ObjectMatrix();
		modelViewMatrix = viewMatrix * modelMatrix;
		glUniformMatrix4fv(ModelView, 1, GL_FALSE, glm::value_ptr(modelViewMatrix));

		ModelViewProjectionMatrix = projectionMatrix * modelViewMatrix;
		glUniformMatrix4fv(MVP, 1, GL_FALSE, glm::value_ptr(ModelViewProjectionMatrix));

		glBindVertexArray(*(entity->ModelFile()->VAO()));
		glDrawArrays(GL_POINTS, 0, entity->ModelFile()->Vertices());
	}

	glutSwapBuffers();
	frameCount++;
	currentTime = glutGet(GLUT_ELAPSED_TIME);
	timeInterval = currentTime - lastTime;

	// Update fps
	if (timeInterval >= 1000)
	{
		sprintf(fpsStr, "  F/S %4d", (int)(frameCount / (timeInterval / 1000.0f)));
		lastTime = currentTime;
		frameCount = 0;
		updateTitle();
	}
}

void update(int value)
{
	glutTimerFunc(scene->TimerDelay(), update, 1);

	scene->Update();

	viewingCamera = scene->ViewingCamera();
	viewMatrix = viewingCamera->ViewMatrix();
	setUniform("cameraPos", glm::vec3(viewMatrix * glm::vec4(viewingCamera->Eye(),1)));

	updateCount++;
	currentTime = glutGet(GLUT_ELAPSED_TIME);
	utimeInterval = currentTime - ulastTime;

	// Update ups
	if (utimeInterval >= 1000)
	{
		sprintf(upsStr, "  U/S %3d", (int)(updateCount / (utimeInterval / 1000.0f)));
		ulastTime = currentTime;
		updateCount = 0;
		updateTitle();
	}
}

// load the shader programs, vertex data from model files, create the solids, set initial view
void init()
{
	// load the shader programs
	shaderProgram = loadShaders(vertexShaderFile, fragmentShaderFile);
	glUseProgram(shaderProgram);

	// generate VAOs and VBOs
	glGenVertexArrays(nModels, VAO);
	glGenBuffers(nModels, buffer);

	// Load models
	for (int i = 0; i < nModels; i++)
	{
		new PointCloud(modelFile[i], &VAO[i], &buffer[i], &shaderProgram);
	}

	StaticEntity* pc = new StaticEntity(scene->GetModel("knot"));
	sprintf(pointClousdStr, "  Point Cloud %s", pc->ModelFile()->File());
	sprintf(verticesStr, "  Vertices %i", pc->ModelFile()->Vertices());

	// Initialize display info
	lastTime = glutGet(GLUT_ELAPSED_TIME);
	ulastTime = lastTime;

	// Get shader program locations
	MVP = glGetUniformLocation(shaderProgram, "MVP");
	ModelView = glGetUniformLocation(shaderProgram, "ModelView");

	new DynamicCamera("Main", pc, false, 0.0f, glm::vec3(0.0f, 0.0f, radius), glm::vec3(0), glm::vec3(0.0f, 1.0f, 0.0f));

	// Set camera
	viewingCamera = scene->ViewingCamera();
	viewMatrix = viewingCamera->ViewMatrix();

	// set render state values
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(1.0);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// Finalize scene
	scene->InitDone();
}

// Keyboard input
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 033:
	case 'q':
	case 'Q':
		exit(EXIT_SUCCESS);
		break;

	case 't':
	case 'T':  // Change time quantum
		tq = (tq + 1) % 4;
		switch (tq)
		{
		case 0:
			scene->SetTimerDelay(5);
			break;
		case 1:
			scene->SetTimerDelay(40);
			break;
		case 2:
			scene->SetTimerDelay(100);
			break;
		case 3:
			scene->SetTimerDelay(500);
			break;
		}
		break;
	}

	updateTitle();
	glutPostRedisplay();
}
void specialKeys(int key, int x, int y)
{
	int modifiers = glutGetModifiers();
	switch (modifiers)
	{
	case GLUT_ACTIVE_CTRL:
		break;
	default:
		break;
	}

}

void specialUpFunc(int key, int x, int y)
{
	int modifiers = glutGetModifiers();
	switch (modifiers)
	{
	case GLUT_ACTIVE_CTRL:
		break;
	default:
		break;
	}
}

void mouseState(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		mouseOldX = x;
		mouseOldY = y;
		rotate = true;
	}
	else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		rotate = false;
	}
}

void mouseMove(int x, int y)
{
	if (rotate)
	{
		theta += (mouseOldX - x) * 0.01f;
		if (abs(theta) >= 2 * PI)
		{
			theta = 0;
		}
		phi += (mouseOldY - y) * 0.01f;
		if (abs(phi) >= 2 * PI)
		{
			phi = 0;
		}
		printf("Theta: %4.4f\n", theta);
		printf("Phi: %4.4f\n", phi);
	}

	mouseOldX = x;
	mouseOldY = y;

	float eyeX = radius * sin(theta) * cos(phi);
	float eyeY = radius * -sin(phi);
	float eyeZ = radius * cos(theta) * cos(phi);

	((DynamicCamera*)viewingCamera)->SetEyeOffset(glm::vec3(eyeX, eyeY, eyeZ));
}

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
# ifdef __Mac__
	// Can't change the version in the GLUT_3_2_CORE_PROFILE
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_3_2_CORE_PROFILE);
# endif
# ifndef __Mac__
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
# endif
	glutInitWindowSize(800, 600);
	// set OpenGL and GLSL versions to 3.3 for Comp 465/L, comment to see highest versions
# ifndef __Mac__
	glutInitContextVersion(3, 3);
	glutInitContextProfile(GLUT_CORE_PROFILE);
# endif
	glutCreateWindow("");
	updateTitle();
	// initialize and verify glew
	glewExperimental = GL_TRUE;  // needed my home system
	GLenum err = glewInit();
	if (GLEW_OK != err)
		printf("GLEW Error: %s \n", glewGetErrorString(err));
	else
	{
		printf("Using GLEW %s \n", glewGetString(GLEW_VERSION));
		printf("OpenGL %s, GLSL %s\n",
			glGetString(GL_VERSION),
			glGetString(GL_SHADING_LANGUAGE_VERSION));
	}

	// Disable vsync
#ifdef _WIN32
	printf("WINDOWS\n");
	if (WGLExtensionSupported("WGL_EXT_swap_control"))
	{
		// Extension is supported, init pointer
		wglSwapIntervalEXT = (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
	}

	wglSwapIntervalEXT(0);
#elif defined __linux__
	printf("LINUX\n");
#endif

	// initialize scene
	init();
	// set glut callback functions
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMouseFunc(mouseState);
	glutMotionFunc(mouseMove);
	glutSpecialFunc(specialKeys);
	glutSpecialUpFunc(specialUpFunc);
	glutTimerFunc(scene->TimerDelay(), update, 1);
	glutIdleFunc(display);
	glutMainLoop();
	printf("done\n");
	delete scene;
	return 0;
}
