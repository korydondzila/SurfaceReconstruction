/*
Kory Dondzila
Garret Richardson

Model.hpp
10/08/2016

Model class, used for keeping track of models used in the
scene, entities have a pointer to a model to know what it
will render.

*/

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "includes/includes.hpp"

class PointCloud
{
private:
    char* m_cFile; // Model file name
    int m_iVertices; // Number of vertices
    GLuint* m_gliVAO;
    GLuint* m_gliBuffer;
    GLuint* m_gliShaderProgram;
    float m_fBoundingRadius; // Bounding radius of model

    // Initialize and load model
    void Init();

public:
	PointCloud(char* modelFile, GLuint* VAO, GLuint* buffer, GLuint* shaderProgram);

    ~PointCloud()
    {
        delete []m_cFile;
    }

    char* File() const { return m_cFile; }
    int Vertices() const { return m_iVertices; }
    GLuint* VAO() const { return m_gliVAO; }
    GLuint* Buffer() const { return m_gliBuffer; }
    GLuint* ShaderProgram() const { return m_gliShaderProgram; }
    float BoundingRadius() const { return m_fBoundingRadius; }
};

#endif
