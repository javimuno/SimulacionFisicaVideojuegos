#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>
#include "Vector3D.h"

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
ContactReportCallback gContactReportCallback;

// RenderItems de la Práctica 0
RenderItem* gOriginSphere = nullptr;
RenderItem* gAxisX = nullptr;
RenderItem* gAxisY = nullptr;
RenderItem* gAxisZ = nullptr;

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

	// ======== PRACTICA 0: ESFERA EN ORIGEN + EJES========
	{
		auto ToPxT = [](const Vector3D& v) { return physx::PxTransform(v.x, v.y, v.z); };

		// distancias y tamaños pensados para que se vean bien con la cámara actual (50,50,50)
		const float dist = 10.0f; // separación desde el origen
		const float rCenter = 1.0f;  // radio esfera central (blanca)
		const float rAxis = 0.8f;  // radio esferas de los ejes

		Vector3D O(0.0f, 0.0f, 0.0f);
		static physx::PxTransform poseO = ToPxT(O);
		gOriginSphere = new RenderItem(CreateShape(physx::PxSphereGeometry(rCenter)), &poseO, { 1,1,1,1 }); // blanca-origen
		RegisterRenderItem(gOriginSphere);

		// Ejes: X (rojo), Y (verde), Z (azul)
		static physx::PxTransform poseX = ToPxT(Vector3D(+dist, 0.0f, 0.0f));
		static physx::PxTransform poseY = ToPxT(Vector3D(0.0f, +dist, 0.0f));
		static physx::PxTransform poseZ = ToPxT(Vector3D(0.0f, 0.0f, +dist));

		gAxisX = new RenderItem(CreateShape(physx::PxSphereGeometry(rAxis)), &poseX, { 1,0,0,1 }); // rojo (X)
		gAxisY = new RenderItem(CreateShape(physx::PxSphereGeometry(rAxis)), &poseY, { 0,1,0,1 }); // verde (Y)
		gAxisZ = new RenderItem(CreateShape(physx::PxSphereGeometry(rAxis)), &poseZ, { 0,0,1,1 }); // azul (Z)

		RegisterRenderItem(gAxisX);
		RegisterRenderItem(gAxisY);
		RegisterRenderItem(gAxisZ);
	}
	// ======== FIN PRACTICA 0 ========

	}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	gScene->simulate(t);
	gScene->fetchResults(true);
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

	// ======== PRACTICA 0: DEREGISTRAR ========
	if (gOriginSphere) { DeregisterRenderItem(gOriginSphere); gOriginSphere = nullptr; }
	if (gAxisX) { DeregisterRenderItem(gAxisX);        gAxisX = nullptr; }
	if (gAxisY) { DeregisterRenderItem(gAxisY);        gAxisY = nullptr; }
	if (gAxisZ) { DeregisterRenderItem(gAxisZ);        gAxisZ = nullptr; }
	// ======== FIN PRACTICA 0 ========
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