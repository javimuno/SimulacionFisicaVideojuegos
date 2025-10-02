#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>
#include "Vector3D.h"
#include "Particle.h"

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

// Partcitula

//Particle* p = nullptr;

// P1: lista de partículas controladas con teclado
std::vector<Particle*> gParticles;


//==============TESTEO PARA SPAWNEAR
static Vector3D nextSpawnPos()
{
	static int idx = 0;
	float x = float((idx % 7) - 3) * 2.0f; // -6,-4,-2,0,2,4,6...
	float y = 0.0f;
	float z = 0.0f;
	idx++;
	return Vector3D(x, y, z);
}

// Spawner genérico para un caso concreto (integrador + damping + vel + acc)
static void spawnParticle(IntegratorType integ, float damping, const Vector3D& vel, const Vector3D& acc)
{
	Vector3D pos = nextSpawnPos();
	Particle* np = new Particle(pos, vel, acc, damping, integ);
	gParticles.push_back(np);
}

//==================================

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


	// === P1 Act 1a: velocidad constante (acc=0, damping=1) ===
	{
		Vector3D startPos(0, 0, 0);
		Vector3D startVel(0.5f, 0.5f, 0.0f);      // la dirección que quieras
		Vector3D accel(0, 0, 0);                  // sin aceleración
		float damping = 1.0f;                     // sin damping (velocidad constante)

		//p = new Particle(startPos, startVel, accel, damping, IntegratorType::EulerSemiImplicit);
		// Puedes probar también el explícito para ver diferencias:
		// p->setIntegrator(IntegratorType::EulerExplicit);
	}

	// === P1 Act 2: aceleración constante ===
	{
		Vector3D startPos(0, 0, 0);
		Vector3D startVel(0.5f, 0.5f, 0.0f);     // tu velocidad inicial de prueba
		Vector3D accel(0.0f, -0.2f, 0.0f);       // aceleración (gravedad suave) _>cambiar aceleracion cambiar comportamiento AQUI JAVI AQUI
		float damping = 1.0f;                    // sin damping aún

		//p = new Particle(startPos, startVel, accel, damping, IntegratorType::EulerSemiImplicit);
	}

	// === P1 Act 3: damping ===
	{
		Vector3D startPos(0, 0, 0);
		Vector3D startVel(0.5f, 0.5f, 0.0f);
		Vector3D accel(2.0f, -0.0f, 2.0f);
		float damping = 0.99f; // 0-1 (0.99 Porque es el valor apropiado para corregir -> teoría Raul Lab)

		//p = new Particle(startPos, startVel, accel, damping, IntegratorType::EulerSemiImplicit);
		// o, si ya la creaste:
		// p->setDamping(0.99f);
	}

	}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	gScene->simulate(t);
	gScene->fetchResults(true);

	//=======P1====
	//if (p) p->integrate(static_cast<float>(t)); // usa el dt real, no 0.3

	//=====CON SPAWNER
	// Integra TODAS con dt real
	for (auto* it : gParticles)
		it->integrate(static_cast<float>(t));
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

	//=====P1===
	//if (p) { delete p; p = nullptr; }

	// P1: liberar partículas creadas a mano
	for (auto* it : gParticles) delete it;
	gParticles.clear();
	}

// Function called when a key is pressed
void keyPress(unsigned char key, const PxTransform& camera)
{
	PX_UNUSED(camera);

	switch(toupper(key))
	{
	//case 'B': break;
	//case ' ':	break;
	case '1': // Euler explícito, SIN damping
	{
		spawnParticle(IntegratorType::EulerExplicit, 1.0f,
			Vector3D(0.5f, 0.5f, 0.0f),
			Vector3D(0.0f, -0.2f, 0.0f));
		display_text = "Spawn: Euler EXP, damping=1.0 (sin damping)";
		break;
	}
	case '2': // Euler explícito, CON damping
	{
		spawnParticle(IntegratorType::EulerExplicit, 0.99f,
			Vector3D(0.5f, 0.5f, 0.0f),
			Vector3D(0.0f, -0.2f, 0.0f));
		display_text = "Spawn: Euler EXP, damping=0.99";
		break;
	}
	case '3': // Euler semi-implícito, SIN damping
	{
		spawnParticle(IntegratorType::EulerSemiImplicit, 1.0f,
			Vector3D(0.5f, 0.5f, 0.0f),
			Vector3D(0.0f, -0.2f, 0.0f));
		display_text = "Spawn: Euler SEMI, damping=1.0 (sin damping)";
		break;
	}
	case '4': // Euler semi-implícito, CON damping
	{
		spawnParticle(IntegratorType::EulerSemiImplicit, 0.99f,
			Vector3D(0.5f, 0.5f, 0.0f),
			Vector3D(0.0f, -0.2f, 0.0f));
		display_text = "Spawn: Euler SEMI, damping=0.99";
		break;
	}
	case 'C': // Clear: borra todas las partículas
	{
		for (auto* it : gParticles) delete it;
		gParticles.clear();
		display_text = "Particulas limpiaditas toas toas toas";
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