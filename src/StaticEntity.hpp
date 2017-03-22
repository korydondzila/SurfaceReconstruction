/*
Kory Dondzila
Garret Richardson

StaticEntity.hpp
10/08/2016

Static entity class, used for displaying static objects in the scene.

*/

#ifndef STATIC_ENTITY_H
#define STATIC_ENTITY_H

#include "PointCloud.hpp"
#include "BaseEntity.hpp"

class StaticEntity : public BaseEntity
{
protected:
	glm::vec3 m_vPosition; // Position of entity
	glm::vec3 m_vForward; // Forward vector of entity
	glm::vec3 m_vLeft; // Left vector of entity
	glm::vec3 m_vUp; // Up vector of entity
	glm::vec3 m_vScale; // Scale vector of entity
	glm::mat4 m_mRotation; // Rotation matrix
	glm::mat4 m_mObject; // Object matrix
	float m_fBoundingRadius; // Bounding radius of entity
	PointCloud* m_pModel; // Model to render

	// Sets rotation matrix for rotating the
	// model to face the forward vector
	void CreateRotationMatrix();

	// Sets the object matrix
	void CreateObjectMatrix();

public:
	StaticEntity(PointCloud* model, const glm::vec3& pos = glm::vec3(), const glm::vec3& scale = glm::vec3(1.0, 1.0, 1.0),
		const glm::vec3& target = glm::vec3(0.0, 0.0, -1.0), const glm::vec3& up = glm::vec3(0.0, 1.0, 0.0));

	virtual ~StaticEntity(){}

	virtual const std::string GetType() const { return "StaticEntity"; }
	virtual bool  HandleMsg(const Message& msg);

	glm::vec3 Position() const { return m_vPosition; }
	virtual void SetPosition(const glm::vec3& position) { m_vPosition = position; }
	virtual void SetPosition(float x, float y, float z) { SetPosition(glm::vec3(x, y, z)); }

	glm::vec3 Scale() const { return m_vScale; }
	void SetScale(float scale) { SetScale(glm::vec3(scale)); }
	void SetScale(float x, float y, float z) { SetScale(glm::vec3(x, y, z)); }
	void SetScale(const glm::vec3& scale)
	{
		m_vScale = scale / m_pModel->BoundingRadius();
		m_fBoundingRadius = glm::max(m_vScale.x, glm::max(m_vScale.y, m_vScale.z)) * m_pModel->BoundingRadius();
	}

	glm::vec3 Forward() const { return m_vForward; }
	glm::vec3 Backward() const { return -m_vForward; }
	glm::vec3 Left() const { return m_vLeft; }
	glm::vec3 Right() const { return -m_vLeft; }
	glm::vec3 Up() const { return m_vUp; }
	glm::vec3 Down() const { return -m_vUp; }

	float BoundingRadius() const { return m_fBoundingRadius; }
	PointCloud* ModelFile() const { return m_pModel; }

	// Returns the rotation matrix
	glm::mat4 RotationMatrix() const { return m_mRotation; }

	// Returns the object matrix
	glm::mat4 ObjectMatrix() const { return m_mObject; }
};

#endif
