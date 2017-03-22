#include "DynamicCamera.hpp"
#include "MoveableEntity.hpp"

DynamicCamera::DynamicCamera(char* name, StaticEntity* parent, bool useHeading, float headingOffset,
                             const glm::vec3& eyeOffset, const glm::vec3& atOffset, const glm::vec3& up, float FOVY,
                             float nearClip, float farClip, bool warpPoint) :
    StaticCamera(name, parent->Position() + eyeOffset, parent->Position() + atOffset,
                 up, FOVY, nearClip, farClip),
    m_pParent(parent),
    m_bUsesHeading(useHeading),
    m_fHeadingOffset(headingOffset),
    m_vEyeOffset(eyeOffset),
    m_vAtOffset(atOffset)
{
	if (useHeading)
	{
		m_vEye = m_fHeadingOffset * ((MoveableEntity*)m_pParent)->Heading() + m_pParent->Position() + m_vEyeOffset;
	}
	else
	{
		m_vEye = m_pParent->Position() + (glm::mat3(m_pParent->RotationMatrix()) * m_vEyeOffset);
	}
    m_vAt = m_pParent->Position() + m_vAtOffset;
    m_mViewMatrix = glm::lookAt(m_vEye, m_vAt, m_vUp);
    Scene::Instance()->AddToDynamicQueue(this);
}

DynamicCamera::DynamicCamera(char* name, StaticEntity* parent, bool useHeading, float headingOffset, bool warpPoint,
                             const glm::vec3& eyeOffset, const glm::vec3& atOffset, const glm::vec3& up, float FOVY,
                             float nearClip, float farClip) :
    StaticCamera(name, parent->Position() + eyeOffset, parent->Position() + atOffset,
                 up, FOVY, nearClip, farClip),
    m_pParent(parent),
    m_bUsesHeading(useHeading),
    m_fHeadingOffset(headingOffset),
    m_vEyeOffset(eyeOffset),
    m_vAtOffset(atOffset)
{
	if (useHeading)
	{
		m_vEye = m_fHeadingOffset * ((MoveableEntity*)m_pParent)->Heading() + m_pParent->Position() + m_vEyeOffset;
	}
	else
	{
		m_vEye = m_pParent->Position() + (glm::mat3(m_pParent->RotationMatrix()) * m_vEyeOffset);
	}
    m_vAt = m_pParent->Position() + m_vAtOffset;
    m_mViewMatrix = glm::lookAt(m_vEye, m_vAt, m_vUp);
    Scene::Instance()->AddToDynamicQueue(this);
}

bool DynamicCamera::HandleMsg(const Message& message)
{
	bool hasMsg = true;

	switch (message.Msg)
	{
	case Msg_DestroySource:
		printf("Msg: Destroy source received by camera ID: %d\n", m_iID);
		if (m_pParent != NULL && message.Sender == m_pParent->ID())
		{
			m_pParent = NULL;
			Scene::Instance()->DestroyEntity(m_iID);
		}
		break;

	default:
		hasMsg = false;
		break;
	}

	return hasMsg;
}

void DynamicCamera::SetEyeOffset(const glm::vec3& offset)
{
	m_vEyeOffset = offset;
}

void DynamicCamera::Update()
{
    // If camera is attached then update
    if (m_pParent != NULL)
    {
        if (m_bUsesHeading) // Uses parent's heading vector
        {
			m_vEye = m_fHeadingOffset * ((MoveableEntity*)m_pParent)->Heading() + m_pParent->Position() + m_vEyeOffset;
            m_vAt = m_pParent->Position() + m_vAtOffset;
        }
        else
        {
            m_vEye = m_pParent->Position() + (glm::mat3(m_pParent->RotationMatrix()) * m_vEyeOffset);
            m_vAt = m_pParent->Position() + (glm::mat3(m_pParent->RotationMatrix()) * m_vAtOffset);
            m_vUp = m_pParent->Up();
        }


        m_mViewMatrix = glm::lookAt(m_vEye, m_vAt, m_vUp);
    }
}
