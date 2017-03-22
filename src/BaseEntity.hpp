/*
Kory Dondzila
Garret Richardson

BaseEntity.hpp
10/08/2016

Base entity class

*/

#ifndef BASE_ENTITY_H
#define BASE_ENTITY_H

#include "Scene.hpp"
#include "MessageDispatcher.hpp"
#include "MessageTypes.hpp"

class BaseEntity
{
protected:
    int m_iID; // Unique ID
    static int s_iNextValidID; // Next valid ID

    void SetID();

public:
    BaseEntity();

    virtual ~BaseEntity() {};

    int ID() const { return m_iID; }
    virtual const std::string GetType() const { return "BaseEntity"; }
	virtual bool HandleMsg(const Message& msg) = 0;
};

#endif
