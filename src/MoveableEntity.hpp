/*
Kory Dondzila
Garret Richardson

MoveableEntity.hpp
10/08/2016

Moveable entity class, used for displaying moveable objects in the scene.
This is meant to be an abstract class.

*/

#ifndef MOVEABLE_ENTITY_H
#define MOVEABLE_ENTITY_H

#include "StaticEntity.hpp"

class MoveableEntity : public StaticEntity
{
protected :
    glm::vec3 m_vHeading; // The direction Entity is moving

public :
    MoveableEntity(PointCloud* model, const glm::vec3& pos = glm::vec3(), const glm::vec3& scale = glm::vec3(1.0f),
                   const glm::vec3& target = glm::vec3(0.0f, 0.0f, -1.0f), const glm::vec3& up = glm::vec3(0.0f, 1.0f, 0.0f)) :
        StaticEntity(model, pos, scale, target, up),
        m_vHeading(glm::vec3(0.0f, 0.0f, -1.0f))
    {
        Scene::Instance()->AddToMoveableQueue(this);
    }

    virtual ~MoveableEntity() {}

    virtual const std::string GetType() const { return "MoveableEntity"; }
	virtual bool HandleMsg(const Message& message) { return false; }

    virtual void Update() = 0;
    virtual glm::vec3 Heading() const { return m_vHeading; }
    virtual void SetHeading(const glm::vec3& heading) { m_vHeading = heading; }
};

#endif
