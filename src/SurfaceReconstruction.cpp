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
#include "Spatial.hpp"
#include "Graph.hpp"
#include "Principal.hpp"
#include "Contour.hpp"
using namespace HuguesHoppe;

// constants for models:  file names, vertex count, model display size
const int nModels = 4;  // number of models in this scene
char * modelFile[nModels] = { "src/knot.pcd" };
const int nVertices[nModels] = { 1280 };
char * vertexShaderFile = "src/simpleVertex.glsl";
char * fragmentShaderFile = "src/simpleFragment.glsl";


// Shader handles, matrices, etc
GLuint shaderProgram;
GLuint MVP, NormalMatrix, ModelView;  // Model View Projection matrix's handle
GLuint VAO[nModels], buffer[nModels];
GLuint vao;

// Point cloud information
PointCloud* pointCloud;  // The loaded point cloud
int numVertices, numContourVertices; // number of vertices/points
std::vector<glm::vec3> points; // The point cloud points
std::vector<glm::vec3> pcTPOrig; // Origins of Tangent Planes
std::vector<glm::mat4x3> pcTP; // Tangent planes
std::vector<glm::vec3> pcTPNorm; // Normals of Tangen Planes
std::vector<bool> pcTPOrient; // Is tangent plane oriented
std::unique_ptr<PointSpatial> SPp; // Point spatial partition
std::unique_ptr<PointSpatial> SPpc; // pcTPOrig spatial partition
std::unique_ptr<Graph<int>> gpcpseudo; // Riemannian on pc centers (based on co)
std::unique_ptr<Graph<int>> gpcpath; // path of orientation propagation
Mesh mesh;
int minkintp = 4, maxkintp = 20, gridsize = 20; // Min/Max number of points in tangent plane
float samplingd = 0.0f; // Sampling density
bool showUnorientTP = false, showOrientTP = false, showContour = false, cullFace = true;

// model, view, projection matrices and values to create modelMatrix.
glm::mat4 modelMatrix;          // set in display()
glm::mat4 modelViewMatrix;
glm::mat4 projectionMatrix;     // set in reshape()
glm::mat4 ModelViewProjectionMatrix; // set in display();

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
float theta = 0, phi = 0, radius = 2.1f;

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

		if (showUnorientTP)
		{
			glBindVertexArray(VAO[1]);
			glDrawArrays(GL_TRIANGLES, 0, 6 * numVertices);
		}

		if (showOrientTP)
		{
			glBindVertexArray(VAO[2]);
			glDrawArrays(GL_TRIANGLES, 0, 6 * numVertices);
		}

		if (showContour)
		{
			glBindVertexArray(VAO[3]);
			glDrawArrays(GL_TRIANGLES, 0, numContourVertices);
		}
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

// Compute the tangent plane
void compute_tp(int i, int& n, glm::mat4x3& f)
{
	std::vector<glm::vec3> pa;
	SpatialSearch ss(*SPp, points[i]);
	for (;;) {
		assert(!ss.done());
		float dis2; int pi = ss.next(&dis2);
		if ((int(pa.size()) >= minkintp && dis2 > square(samplingd)) || int(pa.size()) >= maxkintp) break;
		pa.push_back(points[pi]);
		if (pi != i && !gpcpseudo->contains(i, pi)) gpcpseudo->enter_undirected(i, pi);
	}
	glm::vec3 eimag;
	principal_components(pa, f, eimag);
	n = pa.size();
}

// Compute tangent planes for all points
void process_principal()
{
	for_int(i, numVertices)
	{
		int n;
		glm::mat4x3 f = glm::mat4x3();
		compute_tp(i, n, f);
		pcTP[i] = f;
		pcTPOrig[i] = f[3];
		pcTPNorm[i] = glm::normalize(f[2]);
	}
}

float pc_corr(int i, int j)
{
	if (j == numVertices && i < numVertices) return pc_corr(j, i);
	assert(i >= 0 && j >= 0 && i <= numVertices && j < numVertices);
	float vdot, corr;

	if (i == numVertices)
	{
		vdot = 1.f;     // single exterior link
	}
	else
	{
		vdot = glm::dot(pcTPNorm[i], pcTPNorm[j]);
	}
	corr = 2.f - std::abs(vdot);

	return corr;
}

// Compute the dot product between two tangent planes
float pc_dot(int i, int j)
{
	assert(i >= 0 && j >= 0 && i <= numVertices && j < numVertices);
	if (i == numVertices)
	{
		return pcTPNorm[j][2] < 0.f ? -1.f : 1.f;
	}
	else
	{
		return glm::dot(pcTPNorm[i], pcTPNorm[j]);
	}
}

// Propagate orientation along tree gpcpath from vertex i (orig. num) using recursive DFS.
void propagate_along_path(int i)
{
	assert(i >= 0 && i <= numVertices);
	if (i < numVertices) assert(pcTPOrient[i]);
	for (int j : gpcpath->edges(i))
	{
		assert(j >= 0 && j <= numVertices);
		if (j == numVertices || pcTPOrient[j]) continue; // immediate caller
		float corr = pc_dot(i, j);
		if (corr<0) pcTPNorm[j] = -pcTPNorm[j];
		pcTPOrient[j] = true;
		propagate_along_path(j);
	}
}

void add_exterior_orientation(const std::set<int>& nodes)
{
	// vertex num is a pseudo-node used for outside orientation
	gpcpseudo->enter(numVertices);

	// add 1 pseudo-edge to point with largest z value
	float maxz = -BIGFLOAT;
	int maxi = -1;

	for (int i : nodes)
	{
		if (pcTPOrig[i][2] > maxz) { maxz = pcTPOrig[i][2]; maxi = i; }
	}

	gpcpseudo->enter_undirected(maxi, numVertices);
}

void remove_exterior_orientation()
{
	std::vector<int> ari = std::vector<int>();
	for (int j : gpcpseudo->edges(numVertices)) { ari.push_back(j); }
	for (int i : ari) { gpcpseudo->remove_undirected(numVertices, i); }
	gpcpseudo->remove(numVertices);
}

// Orient the set of tangent planes (orient surface normals)
void orient_set(const std::set<int>& nodes)
{
	printf("component with %d points\n", nodes.size());
	add_exterior_orientation(nodes);
	gpcpath = std::make_unique<Graph<int>>();
	for (int i : nodes) { gpcpath->enter(i); }
	gpcpath->enter(numVertices);
	{
		// must be connected here!
		assert(graph_mst<int>(*gpcpseudo, pc_corr, *gpcpath));
	}
	int nextlink = gpcpath->out_degree(numVertices);
	if (nextlink>1) printf(" num_exteriorlinks_used=%d\n", nextlink);
	propagate_along_path(numVertices);
	gpcpath.reset();
	remove_exterior_orientation();
}

// Orient each tangent plane, by considering neighbors
void orient_tp()
{
	// Now treat each connected component of gpcpseudo separately.
	std::set<int> setnotvis = std::set<int>();
	for_int(i, numVertices) { setnotvis.insert(i); }

	while (!setnotvis.empty())
	{
		std::set<int> nodes = std::set<int>();
		std::queue<int> queue = std::queue<int>();
		int fi = *(setnotvis.begin());
		nodes.insert(fi);
		queue.push(fi);

		while (!queue.empty())
		{
			int i = queue.front();
			queue.pop();
			assert(setnotvis.erase(i));

			for (int j : gpcpseudo->edges(i))
			{
				if (nodes.insert(j).second) queue.push(j);
			}
		}

		orient_set(nodes);
	}

	for_int(i, numVertices) { assert(pcTPOrient[i]); }
}

// Find the closest tangent plane origin and compute the signed distance to that tangent plane.
// Was: check to see if the projection onto the tangent plane lies farther than samplingd from any data point.
// Now: check to see if the sample point is farther than samplingd+cube_size from any data point.
float compute_signed(const glm::vec3& p, glm::vec3& proj)
{
	int tpi;
	{
		SpatialSearch ss(*SPpc, p);
		tpi = ss.next();
	}
	glm::vec3 vptopc = p - pcTPOrig[tpi];
	float dis = glm::dot(vptopc, pcTPNorm[tpi]);
	proj = p - dis * pcTPNorm[tpi];
	if (proj[0] <= 0 || proj[0] >= 1 || proj[1] <= 0 || proj[1] >= 1 || proj[2] <= 0 || proj[2] >= 1)
		return k_Contour_undefined;
	if (1)
	{
		// check that projected point is close to a data point
		SpatialSearch ss(*SPp, proj);
		float dis2; ss.next(&dis2);
		if (dis2>square(samplingd)) return k_Contour_undefined;
	}
	{
		// check that grid point is close to a data point
		SpatialSearch ss(*SPp, p);
		float dis2; ss.next(&dis2);
		float grid_diagonal2 = square(1.f / gridsize)*3.f;
		const float fudge = 1.2f;
		if (dis2>grid_diagonal2*square(fudge)) return k_Contour_undefined;
	}
	return dis;
}

glm::vec3 build_Point(const glm::vec3& p) { return p; }

struct eval_point
{
	float operator()(const glm::vec3& pp) const
	{
		glm::vec3 p = build_Point(pp);
		glm::vec3 proj;
		float dis = compute_signed(p, proj);
		if (dis == k_Contour_undefined) return dis;
		return dis;
	}
};

template<typename Contour> void contour_3D(Contour& contour)
{
	for_int(i, numVertices) { contour.march_from(pcTPOrig[i]); }
}

// Creates the tangent planes for rendering
void makeTangentPlanes(GLuint vao, GLuint vbo)
{
	// Set array sizes
	int vec3Size = 6 * numVertices * sizeof(glm::vec3);
	int vec4Size = 6 * numVertices * sizeof(glm::vec4);
	glm::vec4* vertex = (glm::vec4 *) calloc(vec4Size, sizeof(glm::vec4));
	glm::vec4* color = (glm::vec4 *) calloc(vec4Size, sizeof(glm::vec4));
	glm::vec3* normal = (glm::vec3 *) calloc(vec3Size, sizeof(glm::vec3));

	for_int(i, numVertices)
	{
		glm::mat4x4 lookAt;
		glm::vec3 up;
		float x = pcTPNorm[i].x, y = pcTPNorm[i].y, z = pcTPNorm[i].z;

		// Make sure up is in correct direction (not colinear)
		if (abs(y) > abs(x) && abs(y) > abs(z))
		{
			up = glm::vec3(0, 0, 1);
		}
		else
		{
			up = glm::vec3(0, 1, 0);
		}

		// Width/height of plane
		float size = pointCloud->BoundingRadius() / 10;

		// Lookat rotation matrix and translation matrix
		lookAt = glm::transpose(glm::lookAt(glm::vec3(), pcTPNorm[i], up));
		glm::mat4 translate = glm::translate(glm::mat4(), pcTPOrig[i]);

		// Four corners of plane
		glm::vec4 ul = translate * lookAt * glm::vec4(-size, size, 0.0f, 1.0f);
		glm::vec4 ur = translate * lookAt * glm::vec4(size, size, 0.0f, 1.0f);
		glm::vec4 ll = translate * lookAt * glm::vec4(-size, -size, 0.0f, 1.0f);
		glm::vec4 lr = translate * lookAt * glm::vec4(size, -size, 0.0f, 1.0f);

		// Set each vertex in correct order
		for_int(j, 6)
		{
			vertex[i * 6 + j] = j == 0 ? ul : ((j == 1 || j == 4) ? ur : ((j == 2 || j == 3) ? ll : lr));
			color[i * 6 + j] = glm::vec4(1, 1, 1, 1); // Using depth coloring
			normal[i * 6 + j] = glm::vec3();// pcTPNorm[i]; // If using lighting
		}
	}

	// Set vertex data
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * vec4Size + vec3Size, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vec4Size, vertex);
	glBufferSubData(GL_ARRAY_BUFFER, vec4Size, vec4Size, color);
	glBufferSubData(GL_ARRAY_BUFFER, 2 * vec4Size, vec3Size, normal);
	// set vertex shader variable handles
	GLuint vPosition = glGetAttribLocation(shaderProgram, "vPosition");
	glEnableVertexAttribArray(vPosition);
	glVertexAttribPointer(vPosition, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
	GLuint vColor = glGetAttribLocation(shaderProgram, "vColor");
	glEnableVertexAttribArray(vColor);
	glVertexAttribPointer(vColor, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(vec4Size));
	GLuint vNormal = glGetAttribLocation(shaderProgram, "vNormal");
	glEnableVertexAttribArray(vNormal);
	glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(2 * vec4Size));

	free(vertex);
	free(color);
	free(normal);
}

void makeContour(GLuint vao, GLuint vbo)
{
	std::vector<Face> faces = mesh.facesVector();
	numContourVertices = faces.size() * 3;
	int vec3Size = numContourVertices * sizeof(glm::vec3);
	int vec4Size = numContourVertices * sizeof(glm::vec4);
	glm::vec4* vertex = (glm::vec4 *) calloc(vec4Size, sizeof(glm::vec4));
	glm::vec4* color = (glm::vec4 *) calloc(vec4Size, sizeof(glm::vec4));
	glm::vec3* normal = (glm::vec3 *) calloc(vec3Size, sizeof(glm::vec3));

	int index = 0;

	for (Face face : faces)
	{
		HEdge he = face->herep;
		HEdge start = he;
		do
		{
			Vertex v = he->vert;
			vertex[index] = glm::vec4(v->point, 1.0f);
			color[index] = glm::vec4(1, 1, 1, 1); // Using depth coloring
			normal[index] = glm::vec3();
			he = he->next;
			index++;
		} while (he != start);
	}

	// Set vertex data
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * vec4Size + vec3Size, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vec4Size, vertex);
	glBufferSubData(GL_ARRAY_BUFFER, vec4Size, vec4Size, color);
	glBufferSubData(GL_ARRAY_BUFFER, 2 * vec4Size, vec3Size, normal);
	// set vertex shader variable handles
	GLuint vPosition = glGetAttribLocation(shaderProgram, "vPosition");
	glEnableVertexAttribArray(vPosition);
	glVertexAttribPointer(vPosition, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
	GLuint vColor = glGetAttribLocation(shaderProgram, "vColor");
	glEnableVertexAttribArray(vColor);
	glVertexAttribPointer(vColor, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(vec4Size));
	GLuint vNormal = glGetAttribLocation(shaderProgram, "vNormal");
	glEnableVertexAttribArray(vNormal);
	glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(2 * vec4Size));

	free(vertex);
	free(color);
	free(normal);
}

// Convert sphere coordinates to cartesian coordinates
glm::vec3 sphereToCartesian(float rho, float theta, float phi)
{
	glm::vec3 point = glm::vec3();
	point.x = rho * sin(phi) * cos(theta);
	point.y = rho * sin(phi) * sin(theta);
	point.z = rho * cos(phi);
	return point;
}

// load the shader programs, vertex data from model files, create the solids, set initial view
void init()
{
	// Create random sphere point cloud
	if (false)
	{
		std::vector<glm::vec3> points = std::vector<glm::vec3>();
		srand((unsigned int)time(NULL));
		int numPoints = 1000;
		float rho = 1.0;
		
		for_int(i, numPoints)
		{
			float theta = 2 * PI * ((float)rand() / RAND_MAX);//random range is 0.0 to 1.0
			float phi = acos(2.0f * ((float)rand() / RAND_MAX) - 1.0f);
			glm::vec3 point = sphereToCartesian(rho, theta, phi);
			points.push_back(point);
			writePointCloud("src/sphere.pcd", numPoints, points);
		}
	}

	// load the shader programs
	shaderProgram = loadShaders(vertexShaderFile, fragmentShaderFile);
	glUseProgram(shaderProgram);

	// generate VAOs and VBOs
	glGenVertexArrays(nModels, VAO);
	glGenBuffers(nModels, buffer);

	// Load models
	for (int i = 0; i < 1; i++)
	{
		pointCloud = new PointCloud("src/sphere.pcd", &VAO[i], &buffer[i], &shaderProgram);
	}

	StaticEntity* pc = new StaticEntity(scene->GetModel("sphere"));

	numVertices = pointCloud->Vertices();
	samplingd = INFINITY;
	//samplingd = numVertices / pointCloud->BoundingRadius();
	//samplingd = numVertices / (pow(pointCloud->BoundingRadius(), 2));
	//samplingd = 2.f;
	//samplingd = numVertices / (pow(pointCloud->BoundingRadius(), 3));
	//samplingd = numVertices / ((4.f / 3.f) * PI * pow(pointCloud->BoundingRadius(), 3));
	printf("Sampling Density %3f\n", samplingd);

	sprintf(pointClousdStr, "  Point Cloud %s", pointCloud->File());
	sprintf(verticesStr, "  Vertices %i", numVertices);

	// Initialize tangent plane arrays
	pcTPOrig = std::vector<glm::vec3>(numVertices);
	pcTPNorm = std::vector<glm::vec3>(numVertices);
	pcTPOrient = std::vector<bool>(numVertices, false);
	pcTP = std::vector<glm::mat4x3>(numVertices);
	showVec3("Min", pointCloud->MinBound());
	showVec3("Max", pointCloud->MaxBound());

	// Create spatial partition
	int n = numVertices > 100000 ? 60 : numVertices > 5000 ? 36 : 20;
	SPp = std::make_unique<PointSpatial>(n, pointCloud->MinBound(), pointCloud->MaxBound());
	points = *(pointCloud->Points());
	for_int(i, numVertices) { SPp->enter(i, &points[i]); } // Adds all points to spatial partition

	gpcpseudo = std::make_unique<Graph<int>>();
	for_int(i, numVertices) { gpcpseudo->enter(i); } // Add point index to graph

	double time = glutGet(GLUT_ELAPSED_TIME);
	process_principal(); // Compute the tangent planes
	double end = glutGet(GLUT_ELAPSED_TIME);
	printf("Process Principal: %3f\n", ((end - time) / 1000));

	// Create unoriented tangent planes
	makeTangentPlanes(VAO[1], buffer[1]);

	SPpc = std::make_unique<PointSpatial>(n, pointCloud->MinBound(), pointCloud->MaxBound());
	for_int(i, numVertices) { SPpc->enter(i, &pcTPOrig[i]); } // Add tp origins to spatial partition

	time = glutGet(GLUT_ELAPSED_TIME);
	orient_tp(); // Orient tangent planes
	end = glutGet(GLUT_ELAPSED_TIME);
	printf("Orient Tangent Planes: %3f\n", ((end - time) / 1000));
	gpcpseudo.reset();

	// Create oriented tangent planes
	makeTangentPlanes(VAO[2], buffer[2]);

	time = glutGet(GLUT_ELAPSED_TIME);
	Contour3DMesh<eval_point> contour(gridsize, pointCloud->MinBound(), pointCloud->MaxBound(), &mesh);
	contour_3D(contour);
	end = glutGet(GLUT_ELAPSED_TIME);
	printf("Contour: %3f\n", ((end - time) / 1000));

	// Create oriented tangent planes
	makeContour(VAO[3], buffer[3]);

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
	glEnable(GL_POINT_SIZE);
	glPointSize(2.0);
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

	case '1': // Point cloud only
		showUnorientTP = false;
		showOrientTP = false;
		break;

	case '2': // Toggle unoriented TPs
		showUnorientTP = !showUnorientTP;
		showOrientTP = false;
		break;

	case '3': // Toggle oriented TPs
		showUnorientTP = false;
		showOrientTP = !showOrientTP;
		break;

	case '4': // Toggle mesh contour
		showUnorientTP = false;
		showOrientTP = false;
		showContour = !showContour;

	case 'c':
	case 'C': // Toggle culling
		cullFace = !cullFace;
		if (cullFace)
		{
			glEnable(GL_CULL_FACE);
		}
		else
		{
			glDisable(GL_CULL_FACE);
		}
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
		// Get theta radians, add when camera is world up
		// minus when camera is world down (maintains rotation direction)
		if (viewingCamera->Up() == glm::vec3(0.0f, 1.0f, 0.0f))
		{
			theta += (mouseOldX - x) * 0.01f;
		}
		else
		{
			theta -= (mouseOldX - x) * 0.01f;
		}

		if (abs(theta) >= 2 * PI)
		{
			theta = 0;
		}

		// Get phi radians
		phi += (mouseOldY - y) * 0.01f;
		if (abs(phi) >= 2 * PI)
		{
			phi = 0;
		}

		// Have camera up be world down if it goes over the top of object
		// prevents weird flipping
		if (abs(phi) >= PI / 2 && abs(phi) <= 3 * PI / 2)
		{
			((DynamicCamera*)viewingCamera)->SetUp(glm::vec3(0.0f, -1.0f, 0.0f));
		}
		else if (abs(phi) < PI / 2 || abs(phi) > 3 * PI / 2)
		{
			((DynamicCamera*)viewingCamera)->SetUp(glm::vec3(0.0f, 1.0f, 0.0f));
		}

		// Update old position
		mouseOldX = x;
		mouseOldY = y;

		// Get new position
		float eyeX = radius * sin(theta) * cos(phi);
		float eyeY = radius * -sin(phi);
		float eyeZ = radius * cos(theta) * cos(phi);

		((DynamicCamera*)viewingCamera)->SetEyeOffset(glm::vec3(eyeX, eyeY, eyeZ));
	}
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
