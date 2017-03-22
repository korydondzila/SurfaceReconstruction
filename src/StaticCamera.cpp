#include "StaticCamera.hpp"

StaticCamera::StaticCamera(char* name, glm::vec3 eye, glm::vec3 at, glm::vec3 up, float FOVY, float nearClip,
                           float farClip) :
    BaseEntity(),
    m_fFOVY(FOVY),
    m_fNearClip(nearClip),
    m_fFarClip(farClip)
{
    size_t size = strlen(name) + 1;
    m_cName = new char[size];
    strncpy(m_cName, name, size);

    m_mViewMatrix = glm::lookAt(eye, at, up);

    m_vEye = eye;
    m_vAt = at;
    m_vUp = up;

    UpdateProjectionMatrix(m_iViewportWidth, m_iViewportHeight);

    Scene::Instance()->AddStaticCamera(m_iID);
}

bool StaticCamera::HandleMsg(const Message& message)
{
	return false;
}

glm::mat4 StaticCamera::UpdateProjectionMatrix(int width, int height)
{
    m_iViewportWidth = width;
    m_iViewportHeight = height;
    float aspectRatio = (float)m_iViewportWidth / (float)m_iViewportHeight;
    m_mProjectionMatrix = glm::perspective(m_fFOVY, aspectRatio, m_fNearClip, m_fFarClip);
    return m_mProjectionMatrix;
}
