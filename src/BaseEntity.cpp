#include "BaseEntity.hpp"

int BaseEntity::s_iNextValidID = 0;

BaseEntity::BaseEntity()
{
    SetID();
    Scene::Instance()->AddEntity(this);
}

void BaseEntity::SetID()
{
    m_iID = s_iNextValidID++;
}
