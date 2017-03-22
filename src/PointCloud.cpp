#include "PointCloud.hpp"
#include "Scene.hpp"

PointCloud::PointCloud(char* modelFile,
             GLuint* VAO, GLuint* buffer, GLuint* shaderProgram) :
    m_gliVAO(VAO),
    m_gliBuffer(buffer),
    m_gliShaderProgram(shaderProgram),
    m_fBoundingRadius(0.0)
{
    size_t size = strlen(modelFile) + 1;
    m_cFile = new char[size];
    strncpy(m_cFile, modelFile, size);
    Init();
    Scene::Instance()->AddPointCloud(this);
}

void PointCloud::Init()
{
    float* returned = loadPointCloud(m_cFile,
                                        *m_gliVAO, *m_gliBuffer, *m_gliShaderProgram, GLuint(), GLuint(), GLuint(),
                                        "vPosition", "vColor", "vNormal");
	m_fBoundingRadius = returned[0];
	m_iVertices = (int)returned[1];
}
