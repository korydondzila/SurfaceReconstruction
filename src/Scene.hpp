/*
Kory Dondzila
Garret Richardson

Scene.hpp
10/08/2016

Scene singleton, stores information about what is in the scene,
update delay, entities.

*/

#ifndef SCENE_H
#define SCENE_H

#include "includes/includes.hpp"

class PointCloud;
class BaseEntity;
class StaticEntity;
class MoveableEntity;
class StaticCamera;
class DynamicCamera;

typedef std::map<int, BaseEntity*> EntityMap;
typedef std::set<int> Set;

class Scene
{
private:
    int m_iTimerDelay; // Delay in miliseconds
    bool m_bInit;
    static Scene* s_pInstance;
    std::map<std::string, PointCloud*>* m_pPointClouds; // Pointer to map of models
    EntityMap* m_pEntities; // Hash map of all scene entities
    Set* m_pStaticEntities; // Pointer to set of static entity IDs
	Set* m_pMoveableEntities; // Pointer to set of moveable entity IDs
	std::queue<MoveableEntity*>* m_pMoveableEntitesQueue; // Moveables to add to vector
	Set* m_pStaticCameras; // Pointer to vector of static cameras
	Set* m_pDynamicCameras; // Pointer to vector of dynamic cameras
	std::queue<DynamicCamera*>* m_pDynamicCamerasQueue; // Cameras to add to vector
	Set::iterator m_itViewingCamera; // Current camera iterator
	Set* m_pDestroyedEntities; // Entities that are to be destroyed

    void Preprocess();

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

public:
    Scene()
    {
        m_iTimerDelay = 5;
        m_bInit = true;
		m_pPointClouds = new std::map<std::string, PointCloud*>();
        m_pEntities = new EntityMap();
        m_pStaticEntities = new Set();
		m_pMoveableEntities = new Set();
		m_pMoveableEntitesQueue = new std::queue<MoveableEntity*>();
		m_pStaticCameras = new Set();
		m_pDynamicCameras = new Set();
		m_pDynamicCamerasQueue = new std::queue<DynamicCamera*>();
		m_pDestroyedEntities = new Set();
		m_itViewingCamera = m_pStaticCameras->begin();
    }

    ~Scene();

    static Scene* Instance()
    {
        if (!s_pInstance)
        {
            s_pInstance = new Scene();
        }
        return s_pInstance;
    }

    void Update();

    int TimerDelay()
    {
        return m_iTimerDelay;
    }

    void SetTimerDelay(int delay)
    {
        m_iTimerDelay = delay;
    }

    void InitDone()
    {
        m_bInit = false;
    }

	PointCloud* GetModel(const std::string& name) const
    {
        return m_pPointClouds->at(name);
    }
    void AddPointCloud(PointCloud* model);

    EntityMap* Entities() const
    {
        return m_pEntities;
    }
    BaseEntity* GetEntityFromID(int id) const
    {
        auto e = m_pEntities->find(id);
        if (e != m_pEntities->end())
        {
            return e->second;
        }

        return NULL;
    }
    void AddEntity(BaseEntity* entity);

    Set* DrawableObjects()
    {
        return m_pStaticEntities;
    }
    void AddStaticEntity(int id)
    {
        m_pStaticEntities->insert(id);
    }

	Set* CollidableObjects()
	{
		return m_pMoveableEntities;
	}
	void AddMoveableEntity(int id)
	{
		m_pMoveableEntities->insert(id);
	}
	void AddToMoveableQueue(MoveableEntity* entity);

    void AddStaticCamera(int id)
    {
        if (m_bInit)
        {
            m_pStaticCameras->insert(id);
            m_itViewingCamera = m_pStaticCameras->begin();
        }
        else
        {
            int temp = *m_itViewingCamera;
            m_pStaticCameras->insert(id);
            m_itViewingCamera = m_pStaticCameras->find(temp);
        }
    }

    StaticCamera* ViewingCamera() const
    {
        return (StaticCamera*)m_pEntities->at(*m_itViewingCamera);
    }
    StaticCamera* NextCamera()
    {
        m_itViewingCamera++;
        if (m_itViewingCamera == m_pStaticCameras->end())
        {
            m_itViewingCamera = m_pStaticCameras->begin();
        }

        return ViewingCamera();
    }
    StaticCamera* PrevCamera()
    {
        if (m_itViewingCamera == m_pStaticCameras->begin())
        {
            m_itViewingCamera = m_pStaticCameras->end();
        }

        m_itViewingCamera--;
        return ViewingCamera();
    }

	void AddDynamicCamera(int id)
	{
		m_pDynamicCameras->insert(id);
	}
	void AddToDynamicQueue(DynamicCamera* entity);

	void DestroyEntity(int id)
	{
		m_pDestroyedEntities->insert(id);
	}
};

#endif
