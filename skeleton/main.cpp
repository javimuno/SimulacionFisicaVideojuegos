#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>
#include "Vector3D.h"
#include "Particle.h"
#include <string>


//=========== GLOBALES =========

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

// escala de tiempo ---->> de momento esto no 
float gTimeScale = 1.0f; //multiplicador de dt en la integración


//======================= FIN GLOBALES=================

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

// de la práctica 1.2 
// === Fórmulas de real y simulada
// 
// ms = mr * (vr/vs)^2
// gs = g  * (vs/vr)^2
// 

// === Proyectiles: conversión PxVec3 <-> Vector3D
static Vector3D ToV3(const physx::PxVec3& v) { return Vector3D(v.x, v.y, v.z); }
static physx::PxVec3 ToPx(const Vector3D& v) { return physx::PxVec3(v.x, v.y, v.z); }

struct ProjectileSpec {
	const char* name; // para saber cual es en pantalla y ver los valores
	float m_real;     // kg (masa real)
	float v_real;     // m/s (vel real del proyectil)
	float v_sim;      // m/s (vel simulada)
	float g_real = 9.81f; // m/s^2 (gravedad "real")
	float damping = 0.99f; // damping general
};

static Particle* spawnProjectileFromCamera(const ProjectileSpec& spec, IntegratorType integ)
{
	using namespace physx;
	// camara
	auto* cam = GetCamera(); 
	PxVec3 eye = cam->getEye();
	PxVec3 dir = cam->getDir().getNormalized();

	// Cálculo ms y gs segun las formulas
	const float ms = spec.m_real * (spec.v_real * spec.v_real) / (spec.v_sim * spec.v_sim);
	const float gs = spec.g_real * (spec.v_sim * spec.v_sim) / (spec.v_real * spec.v_real);

	// Pos y vel iniciales
	Vector3D pos = ToV3(eye);
	Vector3D vel = ToV3(dir) * spec.v_sim;

	//  Aceleración de la gravedad (negativas en balas suben) no aqui
	Vector3D acc(0.0f, -gs, 0.0f);

	// Creación de particula con masa simulada
	Particle* p = new Particle(pos, vel, acc, spec.damping, integ, ms);
	gParticles.push_back(p);

	// Texto informativo del proyectil generado
	display_text = std::string("Spawn [") + spec.name + "]: "
		+ "ms=" + std::to_string(ms) + " kg, "
		+ "gs=" + std::to_string(gs) + " m/s^2, "
		+ "vs=" + std::to_string(spec.v_sim) + " m/s";
	return p;
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
	// alterador de tiempo --->> De momento no funciona o no se me ocurre
	float dt = static_cast<float>(t) * gTimeScale;
	// Integra TODAS con dt real
	for (auto* it : gParticles)
		it->integrate(dt);
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
			Vector3D(2.5f, 6.5f, 0.0f),
			Vector3D(0.0f, -9.8f, 0.0f));
		display_text = "Spawn: Euler EXP, damping=1.0 (sin damping)";
		break;
	}
	case '2': // Euler explícito, CON damping
	{
		spawnParticle(IntegratorType::EulerExplicit, 0.90f,
			Vector3D(2.5f, 60.5f, 0.0f),
			Vector3D(0.0f, -9.8f, 0.0f));
		display_text = "Spawn: Euler EXP, damping=0.99";
		break;
	}
	case '3': // Euler semi-implícito, SIN damping
	{
		spawnParticle(IntegratorType::EulerSemiImplicit, 1.0f,
			Vector3D(2.5f, 6.5f, 0.0f),
			Vector3D(0.0f, -9.8f, 0.0f));
		display_text = "Spawn: Euler SEMI, damping=1.0 (sin damping)";
		break;
	}
	case '4': // Euler semi-implícito, CON damping
	{
		spawnParticle(IntegratorType::EulerSemiImplicit, 0.90f,
			Vector3D(2.5f, 60.5f, 0.0f),
			Vector3D(0.0f, -9.8f, 0.0f));
		display_text = "Spawn: Euler SEMI, damping=0.99";
		break;
	}
	case '+': {
		gTimeScale *= 1.5f;            // sube 50%
		if (gTimeScale > 10.0f) gTimeScale = 10.0f;  // cap
		display_text = "TimeScale x" + std::to_string(gTimeScale);
		break;
	}
	case '-': {
		gTimeScale /= 1.5f;            // baja 33%
		if (gTimeScale < 0.1f) gTimeScale = 0.1f;    // floor
		display_text = "TimeScale x" + std::to_string(gTimeScale);
		break;
	}
		case 'C': // Clear: borra todas las partículas
	{
		for (auto* it : gParticles) delete it;
		gParticles.clear();
		display_text = "Particulas limpiaditas toas toas toas";
		break;
	}
		case 'B':  // Bala
		{
			ProjectileSpec S{
				"Bala",
				/* m_real */ 0.008f,     
				/* v_real */ 380.0f,     
				/* v_sim  */ 40.0f,      
				/* g_real */ 9.81f,
				/* damping */ 0.99f
			};
			spawnProjectileFromCamera(S, IntegratorType::EulerSemiImplicit);
			break;
		}
		case 'Z':  // lanza patatas, o pelotas de beisbol
		{
			ProjectileSpec S{
				"Pelota",
				/* m_real */ 0.145f,     // ej. peso medio patata 145 g
				/* v_real */ 55.0f,      // ej. 55 m/s media de un lanzapatatas o 42 m/s beibsol
				/* v_sim  */ 20.0f,
				/* g_real */ 9.81f,
				/* damping */ 0.995f
			};
			spawnProjectileFromCamera(S, IntegratorType::EulerSemiImplicit);
			break;
		}
		case 'H':  // lanza globos
		{
			ProjectileSpec S{
				"Helio",
				/* m_real */ 0.005f,
				/* v_real */ 10.0f,
				/* v_sim  */ 8.0f,
				/* g_real */ -9.81f,     // ponemos negativa para que suba
				/* damping */ 0.99f
			};
			spawnProjectileFromCamera(S, IntegratorType::EulerSemiImplicit);
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