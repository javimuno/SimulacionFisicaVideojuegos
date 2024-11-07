#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>
#include <iostream>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"
#include "Particle.h"
#include "Projectile.h"
#include "Emitter.h"

#include <iostream>


std::string display_text = "This is a testant";


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;


PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;
Particle* p = nullptr; //nueva particula
Particle* p1 = nullptr; //nueva particula Euler Semi
Particle* p2 = nullptr; //nueva particula Verlet
Projectile* projectile = nullptr; //bala
std::vector<Projectile*> projectiles; //array para las balas

Emitter* emitter = nullptr;
Emitter* hoseEmitter = nullptr;
Emitter* fogEmitter = nullptr;
Emitter* explosionEmitter = nullptr;

ContactReportCallback gContactReportCallback;




// Initialize physics engine
void initPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	// For Solid Rigids +++++++++++++++++++++++++++++++++++++
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = contactReportFilterShader;
	sceneDesc.simulationEventCallback = &gContactReportCallback;
	gScene = gPhysics->createScene(sceneDesc);



	//// Crear la geometría de la esfera
	//PxSphereGeometry sphereGeometry(1.0f);  // Esfera con un radio de 1.0

	//// Crear un material para la esfera
	//PxMaterial* sphereMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	//// Crear el shape (forma) usando CreateShape
	//PxShape* sphereShape = CreateShape(sphereGeometry, sphereMaterial);

	//// Crear un actor dinámico (la esfera puede moverse)
	////PxRigidDynamic* sphereActor = gPhysics->createRigidDynamic(PxTransform(PxVec3(0, 10, 0)));  // Posición inicial en (0, 10, 0)

	//// Crear un actor estático (la esfera no se mueve)
	//PxRigidStatic* sphereActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 0, 0)));  // Posición en el origen (0, 0, 0)

	//// Adjuntar la forma al actor
	//sphereActor->attachShape(*sphereShape);

	//// Añadir el actor a la escena física
	//gScene->addActor(*sphereActor);

	//// Crear el render item y registrarlo para que se muestre en la escena gráfica
	//RenderItem* sphereRenderItem = new RenderItem(sphereShape, sphereActor, PxVec4(1.0f, 0.0f, 0.0f, 1.0f));  // Color rojo
	//RegisterRenderItem(sphereRenderItem);



	// 1. Pelota Roja (Origen)
	PxSphereGeometry sphereGeometry(1.0f);
	PxMaterial* sphereMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
	PxShape* sphereShape = CreateShape(sphereGeometry, sphereMaterial);
	PxRigidStatic* sphereActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 0, 0)));
	sphereActor->attachShape(*sphereShape);
	gScene->addActor(*sphereActor);
	RenderItem* sphereRenderItem = new RenderItem(sphereShape, sphereActor, PxVec4(1.0f, 0.0f, 0.0f, 1.0f));  // Color rojo
	RegisterRenderItem(sphereRenderItem);

	// 2. Crear los vectores posición
	Vector3D redPosition(0.0f, 0.0f, 0.0f);  // Pelota roja en el origen
	Vector3D greenPosition = redPosition + Vector3D(0.0f, 10.0f, 0.0f);  // Pelota verde en eje Y
	Vector3D bluePosition = redPosition + Vector3D(0.0f, 0.0f, 10.0f); // Pelota azul en eje Z
	Vector3D whitePosition = redPosition + Vector3D(10.0f, 0.0f, 0.0f); // Pelota blanca en eje X

	// 3. Crear la pelota verde en eje Y
	PxRigidStatic* greenActor = gPhysics->createRigidStatic(PxTransform(PxVec3(greenPosition.x, greenPosition.y, greenPosition.z)));
	greenActor->attachShape(*sphereShape);
	gScene->addActor(*greenActor);
	RenderItem* greenRenderItem = new RenderItem(sphereShape, greenActor, PxVec4(0.0f, 1.0f, 0.0f, 1.0f));  // Color verde
	RegisterRenderItem(greenRenderItem);

	// 4. Crear la pelota azul en eje Z
	PxRigidStatic* blueActor = gPhysics->createRigidStatic(PxTransform(PxVec3(bluePosition.x, bluePosition.y, bluePosition.z)));
	blueActor->attachShape(*sphereShape);
	gScene->addActor(*blueActor);
	RenderItem* blueRenderItem = new RenderItem(sphereShape, blueActor, PxVec4(0.0f, 0.0f, 1.0f, 1.0f));  // Color azul
	RegisterRenderItem(blueRenderItem);

	// 5. Crear la pelota blanca en eje X
	PxRigidStatic* whiteActor = gPhysics->createRigidStatic(PxTransform(PxVec3(whitePosition.x, whitePosition.y, whitePosition.z)));
	whiteActor->attachShape(*sphereShape);
	gScene->addActor(*whiteActor);
	RenderItem* whiteRenderItem = new RenderItem(sphereShape, whiteActor, PxVec4(1.0f, 1.0f, 1.0f, 1.0f));  // Color blanco
	RegisterRenderItem(whiteRenderItem);

	//particula (2)
   //p = new Particle({ 0,0,0 }, { .5,0,0 });

   //particula (3)
	p = new Particle({ 0,0,0 }, { 1,0,0 }, { 0,0.05,0.05 }, 0.99f, true);


	//particulas Euler Semi-implicito y Verlet
	//Particle* p1 = new Particle({ 0,3,0 }, { 0.5,0,0 }, { 0,0.5,0 }, 0.99f,true);  // Esta partícula usa semi-implícito
	//Particle* p2 = new Particle({ 0,0,3 }, { 0.5,0,0 }, { 0,0.5,0 }, 0.99f,true);  // Esta partícula usa Verlet


	//PROYECTIL PRACTICA 1.2
	// 
	//parametros iniciales

	//
	Vector3D initialPos(0.0f, 20.0f, 0.0f);
	Vector3D initialVel(20.0f, 10.0f, 0.0f);  // vel inicial
	Vector3D gravity(0.0f, -9.8f, 0.0f);      // gravedad
	float mass = 1.0f;                        // Ajustar la masa según tu simulación

	projectile = new Projectile(initialPos, initialVel, gravity, mass);

	//EMISORES

	//emitter = new Emitter({ 10, 10, 0 }, { 1, 15, 0 }, 5.0f, 3.0f);  // Configuración del emisor



	// Inicialización de los emisores
	//Vector3D baseVelocityHose(0.0f, 1000.0f, 0.0f);  // Velocidad para la manguera
	//Vector3D baseVelocityFog(0.0f, 1.0f, 0.0f);   // Velocidad para la niebla
	//Vector3D baseVelocityExplosion(0.0f, 2000.0f, 0.0f);  // Velocidad para la explosión

	//Vector3D velocityHose = hoseEmitter->generateGaussianDispersion(baseVelocityHose);
	//Vector3D velocityFog = hoseEmitter->generateGaussianDispersion(baseVelocityFog);
	//Vector3D velocityExplosion = hoseEmitter->generateGaussianDispersion(baseVelocityExplosion);


	//P2

	/*Vector3D velocityAux = hoseEmitter->generateRandomVelocity(30.0f, 1.0f);
	Vector3D velocityFog = fogEmitter->generateRandomVelocity(0.5f, 0.2f);
	Vector3D velocityExplosion = explosionEmitter->generateRandomVelocity(300.0f, 250.0f);


	hoseEmitter = new Emitter({ 0.0f, 0.0f, 0.0f }, velocityAux, 50.0f, 7.0f, 1);

	fogEmitter = new Emitter({ 10.0f, 10.0f, 0.0f }, velocityFog, 50.0f, 5.0f, 2);

	explosionEmitter = new Emitter({ -20.0f, 10.0f, 0.0f }, velocityExplosion, 50.0f, 2.5f, 3);*/


}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	gScene->simulate(t);
	gScene->fetchResults(true);
	//p->integrate(0.01); //Partícula para (2) y (3)
	//p1->integrateSemiImplicitEuler(0.01);  // Partícula Euler (4)
	//p2->integrateVerlet(0.016);  // Partícula Verlet

	//proyectiles creados en ini
	if (projectile) {
		projectile->integrate(t);
	}

	//para disparar proyectiles
	for (auto& proj : projectiles) {
		proj->integrate(t);
	}

	if (emitter) {
		emitter->Update(t);
	}

	if (hoseEmitter) {
		hoseEmitter->Update(t);
	}

	if (fogEmitter) {
		fogEmitter->Update(t * 0.3f); //para que sea más lento pero no cambie el update
	}

	if (explosionEmitter) {
		explosionEmitter->Update(t);
	}
}

// Function to clean data
// Add custom code to the begining of the function
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	//test de limpiar memoria
	if (projectile) {
		delete projectile;
		projectile = nullptr;
	}

	// Rigid Body ++++++++++++++++++++++++++++++++++++++++++
	gScene->release();
	gDispatcher->release();
	// -----------------------------------------------------
	gPhysics->release();
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();

	gFoundation->release();
}

// Function called when a key is pressed
void keyPress(unsigned char key, const PxTransform& camera)
{
	PX_UNUSED(camera);

	switch (toupper(key))
	{
		//case 'B': break;
		//case ' ':	break;
	case '1': {
		// Crear un nuevo proyectil estándar
		Vector3D pos(camera.p.x - 10.0f, camera.p.y, camera.p.z - 10.0f);
		//desde origten
		//Vector3D pos(0, 0, 0);
		Vector3D vel(camera.q.getBasisVector2().x * -350.0f,
			camera.q.getBasisVector2().y * -350.0f,
			camera.q.getBasisVector2().z * -350.0f);
		//Vector3D vel(-330.0, -35.0, -330.0);


		//vel pistola = 330 m/s
		Vector3D sgrav = -9.8 * (vel.x / 33, vel.y / 33, vel.z / 330);



		projectiles.push_back(new Projectile(pos, vel, sgrav, 0.10f));

		break;
	}
	default:
		break;
	}
}

void onCollision(physx::PxActor* actor1, physx::PxActor* actor2)
{
	PX_UNUSED(actor1);
	PX_UNUSED(actor2);
}


int main(int, const char* const*)
{
#ifndef OFFLINE_EXECUTION 
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}