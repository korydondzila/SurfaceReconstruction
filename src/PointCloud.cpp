#include "PointCloud.hpp"
#include "Scene.hpp"

PointCloud::PointCloud(char* modelFile,
             GLuint* VAO, GLuint* buffer, GLuint* shaderProgram) :
    m_gliVAO(VAO),
    m_gliBuffer(buffer),
    m_gliShaderProgram(shaderProgram),
    m_fBoundingRadius(0.0),
	m_vPoints(new std::vector<glm::vec3>()),
	m_vMinBound(glm::vec3()),
	m_vMaxBound(glm::vec3())
{
    size_t size = strlen(modelFile) + 1;
    m_cFile = new char[size];
    strncpy(m_cFile, modelFile, size);
	m_vPoints = new std::vector<glm::vec3>();
    Init();

	// shorten name
	std::string file = modelFile;
	size_t f1 = file.find_last_of('/');
	size_t f2 = file.find_last_of('.');
	file = file.substr(f1 + 1, f2 - f1 - 1);
	size = file.size() + 1;
	delete[] m_cFile;
	m_cFile = new char[size];
	strncpy(m_cFile, file.c_str(), size);

    Scene::Instance()->AddPointCloud(this);
}

void PointCloud::Init()
{
    float* returned = loadPointCloud(m_cFile,
                                        *m_gliVAO, *m_gliBuffer, *m_gliShaderProgram, GLuint(), GLuint(), GLuint(),
                                        m_vPoints, m_vMinBound, m_vMaxBound);
	m_fBoundingRadius = returned[0];
	m_iVertices = (int)returned[1];
}

void PointCloud::OutputPoints()
{
	for each (glm::vec3 point in *m_vPoints)
	{
		showVec3("", point);
	}
}
