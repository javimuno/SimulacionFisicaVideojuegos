#include "RigidWorld.h"
#include <cstdlib>  
#include <cmath>

using namespace physx;

RigidWorld::RigidWorld(PxPhysics* physics, PxScene* scene, PxMaterial* mat)
    : mPhysics(physics), mScene(scene), mMat(mat) {
}

RigidWorld::~RigidWorld() { clear(); }

void RigidWorld::clear() {
    for (auto& it : mItems) {
        if (it.ri) { DeregisterRenderItem(it.ri); delete it.ri; it.ri = nullptr; }
        if (it.pose) { delete it.pose; it.pose = nullptr; }
        if (it.actor) {
            if (mScene) mScene->removeActor(*it.actor);
            it.actor->release();
            it.actor = nullptr;
        }
    }
    mItems.clear();
}

void RigidWorld::update(float dt) {
    for (auto it = mItems.begin(); it != mItems.end(); ) {
        // refrescar pose para el render
        if (it->pose && it->actor) *it->pose = it->actor->getGlobalPose();

        // vida útil compara vida asignada con el tiempo vivido
        if (it->life > 0.0f) {
            it->age += dt;
            if (it->age >= it->life) {
                if (it->ri) { DeregisterRenderItem(it->ri); delete it->ri; }
                if (it->pose) { delete it->pose; }
                if (it->actor) { if (mScene) mScene->removeActor(*it->actor); it->actor->release(); }
                it = mItems.erase(it);
                continue;
            }
        }
        ++it;
    }
}

RigidWorld::RBItem RigidWorld::addRenderForActor(PxRigidActor* a, const PxVec4& color, bool isStatic) {
    PxShape* shape = nullptr;
    a->getShapes(&shape, 1);
    RBItem r{};
    r.actor = a;
    r.pose = new PxTransform(a->getGlobalPose());      // pose en heap
    r.isStatic = isStatic;

    //RenderItem usa (shape, PxTransform*, color)
    r.ri = new RenderItem(shape, r.pose, color);
    RegisterRenderItem(r.ri);

    mItems.push_back(r);
    return r;
}

PxRigidDynamic* RigidWorld::spawnSphere(const PxVec3& c, float radius, float density,
    const PxVec4& color) {
    PxRigidDynamic* body = mPhysics->createRigidDynamic(PxTransform(c));
    PxShape* shp = mPhysics->createShape(PxSphereGeometry(radius), *mMat);
    body->attachShape(*shp);
    shp->release();

    PxRigidBodyExt::updateMassAndInertia(*body, density);
    mScene->addActor(*body);
    addRenderForActor(body, color,false);
    return body;
}

PxRigidDynamic* RigidWorld::spawnBox(const PxVec3& c, const PxVec3& half,
    float density, const PxVec4& color) {
    PxRigidDynamic* body = mPhysics->createRigidDynamic(PxTransform(c));
    PxShape* shp = mPhysics->createShape(PxBoxGeometry(half.x, half.y, half.z), *mMat);
    body->attachShape(*shp);
    shp->release();

    PxRigidBodyExt::updateMassAndInertia(*body, density);
    mScene->addActor(*body);
    addRenderForActor(body, color, /*static*/false);
    return body;
}

void RigidWorld::spawnDebrisExplosion(const physx::PxVec3& center, int count, float v0)
{
    using namespace physx;

    auto frand = [](float a, float b) { return a + (b - a) * (rand() / (float)RAND_MAX); };
    auto rand01 = []() { return rand() / (float)RAND_MAX; };

    for (int i = 0; i < count; ++i) {
        // color aleatorio para ver bien las piezas
        PxVec4 col(0.35f + 0.65f * rand01(),
            0.35f + 0.65f * rand01(),
            0.35f + 0.65f * rand01(), 1.0f);

        // ligera dispersión de origen para evitar overlap
        PxVec3 c = center + PxVec3(frand(-0.10f, 0.10f),
            frand(-0.10f, 0.10f),
            frand(-0.02f, 0.02f));

        // vida 4–8 s para auto-limpieza
        const float life = frand(4.0f, 8.0f);

        // 50% cubos 50% bolitas
        const bool makeSphere = (i % 2 == 0);

        PxRigidDynamic* body = nullptr;
        if (makeSphere) {
            // esfera: radio aleatorio pequeñito
            const float r = frand(0.08f, 0.14f);
            const float density = 300.0f; // mitad que la caja para variar inercia y que se vean mejor ambos
            body = spawnSphere(c, r, density, col); 
        }
        else {
            // caja: medidas aleatorias pero con poca variación y pequeñas tabmien
            const float hx = frand(0.08f, 0.24f);
            const float hy = frand(0.08f, 0.24f);
            const float hz = frand(0.08f, 0.24f);
            const float density = 600.0f;
            body = spawnBox(c, PxVec3(hx, hy, hz), density, col); // spawnBox
        }

        // marca TTL para que desaparezcan
        if (auto r = findItemByActor(body)) r->life = life;

        // impulso radial y pequeño giro
        const float ang = (float)i / (float)count * 6.2831853f;
        PxVec3 dir(cosf(ang), sinf(ang), 0.0f);

        body->addForce(v0 * dir * body->getMass(), PxForceMode::eIMPULSE);
        body->addTorque(PxVec3(frand(-2.0f, 2.0f),
            frand(-2.0f, 2.0f),
            frand(-2.0f, 2.0f)), PxForceMode::eIMPULSE);
    }
}

void RigidWorld::setContext(PxPhysics* p, PxScene* s, PxMaterial* m) {
    mPhysics = p; mScene = s; mMat = m;
}

RigidWorld::RBItem* RigidWorld::findItemByActor(physx::PxRigidActor* a) {
    for (auto& it : mItems)
        if (it.actor == a) return &it;
    return nullptr;
}
