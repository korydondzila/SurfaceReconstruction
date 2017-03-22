/*
Kory Dondzila
Garret Richardson

StaticCamera.hpp
10/08/2016

Static camera class handles non moving cameras.

*/

#ifndef STATIC_CAMERA_H
#define STATIC_CAMERA_H

#include "includes/includes.hpp"
#include "BaseEntity.hpp"

class StaticCamera : public BaseEntity
{
protected :
    glm::mat4 m_mViewMatrix; // Camera view matrix
    glm::mat4 m_mProjectionMatrix; // Camera projection matrix
    glm::vec3 m_vEye; // Camera eye vector
    glm::vec3 m_vAt; // Camera at vector
    glm::vec3 m_vUp; // Camera up vector

    //viewport width and height for determining aspect ratio
    //we set a default, but this will be wiped out by the reshapeFunc
    int m_iViewportWidth = 800; // Viewport width
    int m_iViewportHeight = 600; // Viewport height
    float m_fFOVY; // Field of view
    float m_fNearClip; // Near clipping plane
    float m_fFarClip; // Far clipping plane
    char* m_cName; // Name of camera

public :
    StaticCamera(char* name, glm::vec3 eye, glm::vec3 at, glm::vec3 up, float FOVY = glm::radians(60.0f),
                 float nearClip = 0.1f, float farClip = 10000000.0f);

    virtual ~StaticCamera()
    {
        delete []m_cName;
    }

    virtual const std::string GetType() const { return "StaticCamera"; }
	virtual bool  HandleMsg(const Message& msg);

    glm::mat4 ViewMatrix() const { return m_mViewMatrix; }
    glm::mat4 ProjectionMatrix() const { return m_mProjectionMatrix; }
    glm::mat4 UpdateProjectionMatrix(int width, int height);
	glm::vec3 Eye() const { return m_vEye; }
	glm::vec3 At() const { return m_vAt; }
	glm::vec3 Up() const { return m_vUp; }
    float NearClip() const { return m_fNearClip; }
    void SetNearClip(float nearClip) { m_fNearClip = nearClip; }
    float FarClip() const { return m_fFarClip; }
    void SetFarClip(float farClip) { m_fFarClip = farClip; }
    float FOVY() const { return m_fFOVY; }
	void SetFOVY(float newFOVY) { m_fFOVY = newFOVY; }
    char* Name() const { return m_cName; }
};

#endif
