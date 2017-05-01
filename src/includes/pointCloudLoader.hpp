/*
triModel465.hpp

Two utility functions:  loadTriModel(...) and loadModelBuffer(...)

Using loadModelBuffer(...) in your OpenGL core application will call
loadTriModel(...) to read the model's file data.

LoadTriModel(...) reads a *.tri model file exported from AC3D and creates
glm::vec4 arrays for vertex and color and a glm::vec3 array normal for
surface data.  A vec4 color value and a vec3 normal value is computed
for every vertex (not surface).

The 3 glm::vec? parameters are empty on call and with values
on return.

Function's 4 assumptions:
1.  the *.tri model is centered at (0,0,0).
2.  The *.tri model has only triangles, no co-linear edges/triangle
2.  the *.tri model's vertices have been optimized ( Object | OptimizeVertices ... )
3.  the *.tri model's surfaces have been optimized ( Object | OptimizeSurfaces ... )

Use loadModelBuffer(...) to set *.tri model data into vao's vbo buffer.

Functions prints various error messages, with error returns -1.0f
Functions returns the bounding radius of the model with valid model file.

Note use of std::abs(...) instead of abs(...)

This is for gnu's g++ compiler.  It does not provide abs overloaded
for floats.  abs(float) implicitly converts to abs(int) w/o warning!
Models with a bounding 1 > radius > 0 will truncate to a bounding
radius of 0.0 and so not display.

Mike Barnes
10/11/2013
*/

#ifndef __POINT_CLOUD_LOADER__
#define __POINT_CLOUD_LOADER__

#include "includes.hpp"

inline float* loadPointCloud(char* fileName,
	GLuint vao, GLuint vbo, GLuint shaderProgram,
	GLuint vPosition, GLuint vColor, GLuint vNormal,
	std::vector<glm::vec3>* points, glm::vec3& min, glm::vec3& max) 
{
	const int X = 0, Y = 1, Z = 2;
	FILE * fileIn;
	glm::vec3 point;
	glm::vec4 pColor = glm::vec4(1, 1, 1, 1);
	bool done;
	float coord[3];  // points's x, y, z values
	float maxAxes[3] = { -1000.0f, -1000.0f, -1000.0f }; // maximum lenght of x, y, and z from center
	int count = 0, vertexCount = 0, normalCount = 0, colorCount = 0;
	int* nVertices = new int(-1);

	min = glm::vec3(float(INT_MAX));
	max = glm::vec3(float(INT_MIN));

	// set dynamic array sizes
	int vec3Size = 0;
	int vec4Size = 0;
	// allocate dynamic arrays
	glm::vec4* vertex = NULL;
	glm::vec4* color = NULL;
	glm::vec3* normal = NULL;

	fileIn = fopen(fileName, "r");
	if (fileIn != NULL)
	{
		while (!feof(fileIn))
		{
			done = false;
			if (*nVertices == -1)
			{
				if (fscanf(fileIn, "%i", nVertices) == EOF)
				{
					printf("loadPointCloud error: no vertices defined %s\n", fileName);
					float* toReturn = new float[2]{ -1.0f, -1.0f };
					return toReturn;  // just to satisfy the compiler
				}
				vec3Size = *nVertices * sizeof(glm::vec3);
				vec4Size = *nVertices * sizeof(glm::vec4);
				vertex = (glm::vec4 *) calloc(vec4Size, sizeof(glm::vec4));
				color = (glm::vec4 *) calloc(vec4Size, sizeof(glm::vec4));
				normal = (glm::vec3 *) calloc(vec3Size, sizeof(glm::vec3));
			}

			for (int i = 0; i < 3; i++)
			{
				coord[i] = -99999.9f;
				if (fscanf(fileIn, "%f", &coord[i]) == EOF) // at EOF
				{
					done = true;
					break;
				}
			}

			if (!done)
			{
				// create vertices and normals
				// std::abs(....) is used instead of abs(...) because g++ does not provide float abs(float)
				point = glm::vec3(coord[X], coord[Y], coord[Z]);
				points->push_back(point);
				// update maxAxes for model's bounding sphere
				for_int(i, 3)
				{
					if (maxAxes[i] < std::abs(coord[i])) maxAxes[i] = std::abs(coord[i]);
					if (min[i] > coord[i]) min[i] = coord[i];
					if (max[i] < coord[i]) max[i] = coord[i];
				}
				
				vertexCount++;
				if (vertex != NULL) vertex[vertexCount] = glm::vec4(point.x, point.y, point.z, 1.0f);
				if (color != NULL) color[colorCount++] = pColor;
				if (normal != NULL) normal[normalCount++] = glm::vec3(0, 0, 1);
				count++;
			}  // ! done
			   // create vertexs, normal, color 
		} // while ! eof
	} // if
	else // file is null
	{
		printf("loadPointCloud error:  can't open %s\n", fileName);
		float* toReturn = new float[2]{ -1.0f, -1.0f };
		return toReturn;  // just to satisfy the compiler
	}

	if (vertexCount != *nVertices) {
		printf("loadPointCloud error:  count of vertices mismatch:  vertexCount %4d != nVertices %4d\n",
			vertexCount, *nVertices);
		float* toReturn = new float[2]{ -1.0f, (float)*nVertices };
		return toReturn;  // just to satisfy the compiler
	}
	// bounding radius
	float boundingRadius = -1;
	if (maxAxes[X] >= maxAxes[Y] && maxAxes[X] >= maxAxes[Z]) boundingRadius = maxAxes[X];
	if (maxAxes[Y] >= maxAxes[X] && maxAxes[Y] >= maxAxes[Z]) boundingRadius = maxAxes[Y];
	if (maxAxes[Z] >= maxAxes[X] && maxAxes[Z] >= maxAxes[Y]) boundingRadius = maxAxes[Z];

	if (boundingRadius == -1.0f)
	{
		printf("loadPointCloud error:  returned -1.0f \n");
		float* toReturn = new float[2]{ -1.0f, (float)*nVertices };
		return toReturn;  // just to satisfy the compiler
	}
	else
		printf("loaded %s model with %7.2f bounding radius and %d vertices\n", fileName, boundingRadius, *nVertices);

	// fill the point cloud's buffer
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, 2 * vec4Size + vec3Size, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vec4Size, vertex);
	glBufferSubData(GL_ARRAY_BUFFER, vec4Size, vec4Size, color);
	glBufferSubData(GL_ARRAY_BUFFER, 2 * vec4Size, vec3Size, normal);
	// set vertex shader variable handles
	vPosition = glGetAttribLocation(shaderProgram, "vPosition");
	glEnableVertexAttribArray(vPosition);
	glVertexAttribPointer(vPosition, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
	vColor = glGetAttribLocation(shaderProgram, "vColor");
	glEnableVertexAttribArray(vColor);
	glVertexAttribPointer(vColor, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(vec4Size));
	vNormal = glGetAttribLocation(shaderProgram, "vNormal");
	glEnableVertexAttribArray(vNormal);
	glVertexAttribPointer(vNormal, 3, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(2 * vec4Size));

	printf("vPosition %d, vColor %d, vNormal %d \n", vPosition, vColor, vNormal);
	// reclaim dynamically allocated memory
	free(vertex);
	free(color);
	free(normal);
	float* toReturn = new float[2]{ boundingRadius, (float)*nVertices };
	return toReturn;  // just to satisfy the compiler
}

#endif
