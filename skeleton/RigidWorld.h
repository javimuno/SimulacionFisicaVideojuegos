#pragma once
#include <vector>
#include "PxPhysicsAPI.h"
#include "RenderUtils.hpp"   // RenderItem, RegisterRenderItem, DeregisterRenderItem

class RigidWorld {
public:
    RigidWorld(physx::PxPhysics* physics, physx::PxScene* scene, physx::PxMaterial* mat);
    ~RigidWorld();

    void clear();
    void update(float dt);

    // Spawners dinámicos
    physx::PxRigidDynamic* spawnSphere(const physx::PxVec3& c, float radius, float density,
        const physx::PxVec4& color);
    physx::PxRigidDynamic* spawnBox(const physx::PxVec3& c, const physx::PxVec3& half,
        float density, const physx::PxVec4& color);

    // Explosión de “metralla” (cubitos vaya)
    void spawnDebrisExplosion(const physx::PxVec3& center, int count = 28, float v0 = 14.0f);
    void setContext(physx::PxPhysics* p, physx::PxScene* s, physx::PxMaterial* m);


private:
    struct RBItem {
        physx::PxRigidActor* actor = nullptr;
        physx::PxTransform* pose = nullptr;   //  pose en heap para el renderer
        RenderItem* ri = nullptr;   //  RenderItem atado a (shape, pose)
        bool                 isStatic = false;
        float age = 0.0f;   // tiempo vivido
        float life = -1.0f;  // vida en s; <0 = infinito
    };

    RBItem addRenderForActor(physx::PxRigidActor* a, const physx::PxVec4& color, bool isStatic);
    RBItem* findItemByActor(physx::PxRigidActor* a); //rastreator

    physx::PxPhysics* mPhysics = nullptr;
    physx::PxScene* mScene = nullptr;
    physx::PxMaterial* mMat = nullptr;
    std::vector<RBItem> mItems;

   
};
