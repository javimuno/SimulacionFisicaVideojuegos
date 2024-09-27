#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"
#include "Particle.h"

#include <iostream>


std::string display_text = "This is a test";


using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;


PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene      = NULL;
Particle* p = NULL; //nueva particula

ContactReportCallback gContactReportCallback;




// Initialize physics engine
void initPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

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


	p = new Particle({ 0,0,0 }, { .5,0,0 });


	}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	gScene->simulate(t);
	gScene->fetchResults(true);
	p->integrate(.003);
}

// Function to clean data
// Add custom code to the begining of the function
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);

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

	switch(toupper(key))
	{
	//case 'B': break;
	//case ' ':	break;
	case ' ':
	{
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


int main(int, const char*const*)
{
#ifndef OFFLINE_EXECUTION 
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}