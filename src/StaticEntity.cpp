#include "StaticEntity.hpp"

StaticEntity::StaticEntity(PointCloud* model, const glm::vec3& pos, const glm::vec3& scale, const glm::vec3& target,
	const glm::vec3& up) :
	BaseEntity(),
	m_vPosition(pos),
	m_pModel(model)
{
	SetScale(scale);
	m_vForward = glm::normalize(target - pos);
	m_vLeft = glm::normalize(glm::cross(up, m_vForward));
	m_vUp = glm::normalize(glm::cross(m_vForward, m_vLeft));
	CreateRotationMatrix();
	CreateObjectMatrix();
	Scene::Instance()->AddStaticEntity(m_iID);
}

bool StaticEntity::HandleMsg(const Message& message)
{
	return false;
}

void StaticEntity::CreateRotationMatrix()
{
	m_mRotation = glm::transpose(glm::lookAt(glm::vec3(), m_vForward, m_vUp));
}

void StaticEntity::CreateObjectMatrix()
{
	m_mObject = glm::translate(glm::mat4(), m_vPosition) * m_mRotation * glm::scale(glm::mat4(), m_vScale);
}
