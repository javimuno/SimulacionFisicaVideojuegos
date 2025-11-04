#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <ctype.h>
#include <PxPhysicsAPI.h>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include "Vector3D.h"
#include "Particle.h"
#include "SimpleEmitter.h"
#include "WorldBounds.h"
#include "ForceGenerator.h"
#include "GravityFG.h"
#include "ForceRegistry.h"
#include "WindFG.h"
#include "WhirlwindFG.h"
#include "ZoneSphere.h"
#include "ExplosionFG.h"
#include "Projectile.h"
#include "ParticleSystem.h"


//=========== GLOBALES =========

//std::string display_text = "This is a test";
// Crea los límites una vez
WorldBounds gWorld(Vector3D(-200, -50, -200), Vector3D(200, 200, 200));

//--fuerzas--
ForceRegistry* gForceReg = nullptr;
//--gravedad--
GravityFG* gGravity = nullptr;
//--viento--
WindFG* gWind = nullptr;
//--torbellino--
WhirlwindFG* gWhirl = nullptr;
//--expllosion--
ExplosionFG* gExpl = nullptr;

//para aplicación de fuerzas de impulso en vez de velocidades base

namespace {
	class TimedKickFG : public ForceGenerator {
	public:
		TimedKickFG(const Vector3D& d, float N, float dur)
			: magN_(N), timeLeft_(dur) {
			const float n = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
			dir_ = (n > 0.0f) ? Vector3D(d.x / n, d.y / n, d.z / n) : Vector3D(0, 0, 0);
		}
		void updateForce(Particle* p, float dt) override {
			if (timeLeft_ <= 0.0f) return;   // no toca p cuando expira
			p->addForce(dir_ * magN_);
			timeLeft_ -= dt;
		}
	private:
		Vector3D dir_; float magN_; float timeLeft_;
	};
}
//lo mismo pero para emisores
namespace {
	class TimedKickFG_Emit : public ForceGenerator {
	public:
		TimedKickFG_Emit(const Vector3D& d, float N, float dur)
			: magN_(N), timeLeft_(dur) {
			const float n = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
			dir_ = (n > 0.0f) ? Vector3D(d.x / n, d.y / n, d.z / n) : Vector3D(0, 0, 0);
		}
		void updateForce(Particle* p, float dt) override {
			if (timeLeft_ <= 0.0f) return;
			p->addForce(dir_ * magN_);
			timeLeft_ -= dt;
		}
	private:
		Vector3D dir_; float magN_; float timeLeft_;
	};
}



//=========== MODO ===========
enum class Mode { Projectiles, Emitters };
static Mode gMode = Mode::Projectiles;

//=========== HUD ============
std::string display_text = "P:Projectiles  E:Emitters";

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


// escala de tiempo ---->> de momento esto no 
float gTimeScale = 1.0f; //multiplicador de dt en la integración

// Emitters (3 emisores) + helpers
static ParticleSystem* gEmitSys = nullptr;
static SimpleEmitter* gEmit1 = nullptr;
static SimpleEmitter* gEmit2 = nullptr;
static SimpleEmitter* gEmit3 = nullptr;
static std::vector<Particle*> gProjectiles;

static void ClearProjectiles() {
	for (auto* p : gProjectiles) {
		if (gForceReg) {
			if (gGravity) gForceReg->remove(p, gGravity);
			if (gWind)    gForceReg->remove(p, gWind);
			if (gWhirl)   gForceReg->remove(p, gWhirl);
			if (gExpl)    gForceReg->remove(p, gExpl);
		}
		delete p;
	}
	gProjectiles.clear();
}

static void ClearEmitters() {
	if (gEmit1) gEmit1->clear();
	if (gEmit2) gEmit2->clear();
	if (gEmit3) gEmit3->clear();
}

static void SetMode(Mode m) {
	if (gMode == m) return;
	if (m == Mode::Projectiles) {
		// volvemos a proyectiles: apagamos y limpiamos emisores
		if (gEmit1) gEmit1->setActive(false);
		if (gEmit2) gEmit2->setActive(false);
		if (gEmit3) gEmit3->setActive(false);
		ClearEmitters();
		display_text = "Projectiles: B/Z/H, C  |  +/- time";
	}
	else { // Emitters
		// vamos a emisores: limpiamos proyectiles
		ClearProjectiles();
		display_text = "Emitters: 1/2/3 toggle, [ ] rate, C clear  |  +/- time";
	}
	gMode = m;
}

//======================= FIN GLOBALES=================



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

//SPAWN DE PROYECTILES_______
static void SpawnProjectile(float speed, float damping,
	const physx::PxVec4& color, float radius, float mass)
{
	auto* cam = GetCamera();
	const auto eye = cam->getEye();
	const auto dir = cam->getDir().getNormalized();

	Vector3D pos(eye.x, eye.y, eye.z);
	Vector3D vel(0, 0, 0);
	Vector3D acc(0.0f, 0.0f, 0.0f);


	ProjectileType ptype = (mass >= 2.0f) ? ProjectileType::Heavy : ProjectileType::Light;
	Particle* p = new Projectile(
		pos, vel, acc,
		damping,
		IntegratorType::EulerSemiImplicit,
		mass,
		color,
		radius,
		ptype
	);	

	gProjectiles.push_back(p);
	if (gForceReg && gGravity) gForceReg->add(p, gGravity);
	if (gForceReg && gWind)    gForceReg->add(p, gWind);
	if (gForceReg && gWhirl)   gForceReg->add(p, gWhirl);
	if (gExpl && gExpl->isActive() && gForceReg) gForceReg->add(p, gExpl);

	// Impulso por fuerza durante T para replicar 'speed' (sin v0 a dedo)
	const float T = 0.12f;                           // duración del empuje (s)
	/*const float N = p->getMass() * speed / T;  */      // F = m * (Av/T), con Av = speed
	//con una fuerza fija como esta influye la masa
	const float N = 180.0f;
	Vector3D dirCam(dir.x, dir.y, dir.z);
	auto* kick = new TimedKickFG(dirCam, N, T);
	if (gForceReg) gForceReg->add(p, kick);
}


//helper para explosiones para poder reutilizarla 

static void TriggerExplosionAt(const Vector3D& c) {
	if (!gExpl) return;
	gExpl->setCenter(c);
	gExpl->setParams(/*R*/6.0f, /*K*/800.0f, /*tau*/0.8f);
	gExpl->trigger();

	if (!gForceReg) return;
	for (auto* p : gProjectiles) {
		gForceReg->remove(p, gExpl);
		gForceReg->add(p, gExpl);
	}
	if (gEmit1) gEmit1->registerForceForAlive(gForceReg, gExpl);
	if (gEmit2) gEmit2->registerForceForAlive(gForceReg, gExpl);
	if (gEmit3) gEmit3->registerForceForAlive(gForceReg, gExpl);
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

	//setup de fuerzas-> IMPORTANTE SE DENOTA LA FUERZA AQUI 
	// 
	//Registro de fuerzas
	gForceReg = new ForceRegistry();
	//gravedad
	gGravity = new GravityFG(Vector3D(0.0f, -9.8f, 0.0f));
	//Viento
	gWind = new WindFG(Vector3D(8.0f, 0.0f, 0.0f), /*k1*/ 2.0f, /*k2*/ 0.0f); // viento suave +X
	// Torbellino: esfera centro (2,4,0), radio 2
	ZoneSphere whirlZone(Vector3D(2.0f, 4.0f, 0.0f), 20.0f);
	// Parámetros: omega, updraft, radialIn, k1, k2
	gWhirl = new WhirlwindFG(whirlZone,
		/*omega*/   3.0f, // giro tangencial
		/*updraft*/ 20.0f, //elevacion
		/*radialIn*/2.0f, // fuerza de succion al centro si es 0 no hay o esta en el centro
		/*k1*/      2.0f,  // (lineal)- > laminar
		/*k2*/      0.0f);  //mas fuerte cuando mayor es vRel (turbulento)

	//Explosion
	gExpl = new ExplosionFG(Vector3D(0, 0, 0), /*R*/6.0f, /*K*/800.0f, /*tau*/0.8f);






	// ======== PRACTICA 0: ESFERA EN ORIGEN + EJES========
	{
		auto ToPxT = [](const Vector3D& v) { return physx::PxTransform(v.x, v.y, v.z); };

		// distancias y tamaños 
		const float dist = 10.0f; // separación desde el origen
		const float rCenter = 1.0f;  // radio esfera central (blanca)
		const float rAxis = 0.8f;  // radio esferas de los ejes

		Vector3D O(0.0f, 0.0f, 0.0f);
		static physx::PxTransform poseO = ToPxT(O);
		gOriginSphere = new RenderItem(CreateShape(physx::PxSphereGeometry(rCenter)),
			&poseO, { 1,1,1,1 }); // blanca-origen
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

		
	}

	// === P1 Act 2: aceleración constante ===
	{
		Vector3D startPos(0, 0, 0);
		Vector3D startVel(0.5f, 0.5f, 0.0f);     // velocidad inicial de prueba
		Vector3D accel(0.0f, -0.2f, 0.0f);       // aceleración (gravedad suave) _>cambiar aceleracion cambiar comportamiento AQUI JAVI AQUI
		float damping = 1.0f;                    // sin damping aún

		
	}

	// === P1 Act 3: damping ===
	{
		Vector3D startPos(0, 0, 0);
		Vector3D startVel(0.5f, 0.5f, 0.0f);
		Vector3D accel(2.0f, -0.0f, 2.0f);
		float damping = 0.99f; // 0-1 (0.99 Porque es el valor apropiado para corregir -> teoría Raul Lab)

		
	}

	//===========EMISORES=======================

	{
		SimpleEmitterConfig c1; // Azul: fuente suave
		c1.position = { 0,0,0 };
		c1.posJitter = { 3.15f,1.15f,2.15f };
		c1.speed = 2.0f;
		c1.lifetime = 6.0f;
		c1.damping = 0.995f;
		c1.color = { 0.2f,0.6f,1.0f,1.0f };
		c1.mass = 1.0f;
		c1.radius = 0.32f;
		c1.rate = 6.0f;     // pps
		c1.maxAlive = 250;
		c1.active = false;
		gEmit1 = new SimpleEmitter(c1);

		SimpleEmitterConfig c2; // Blanco: niebla ligera
		c2.position = { 0,2,0 };
		c2.posJitter = { 4.0f,0.4f,4.0f };
		c2.speed = 1.2f;
		c2.lifetime = 6.5f;
		c2.damping = 0.995f;
		c2.color = { 1,1,1,1 };
		c2.mass = 1.0f;
		c2.radius = 0.38f;
		c2.rate = 8.0f;
		c2.maxAlive = 200;
		c2.active = false;
		gEmit2 = new SimpleEmitter(c2);

		SimpleEmitterConfig c3; // Amarillo: chispas
		c3.position = { 3,3,0 };
		c3.posJitter = { 3.08f,1.08f,2.08f };
		c3.speed = 1.0f;
		c3.lifetime = 6.6f;
		c3.damping = 0.995f;
		c3.color = { 1.0f,0.824f,0.2f,1.0f };
		c3.mass = 1.1f;
		c3.radius = 0.37f;
		c3.rate = 7.0f;
		c3.maxAlive = 180;
		c3.active = false;
		gEmit3 = new SimpleEmitter(c3);
	}

	gEmitSys = new ParticleSystem(&gWorld);
	gEmitSys->addEmitter(gEmit1);
	gEmitSys->addEmitter(gEmit2);
	gEmitSys->addEmitter(gEmit3);

	SetMode(Mode::Projectiles);

}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	gScene->simulate(t);
	gScene->fetchResults(true);
	
	// alterador de tiempo --->> De momento no funciona o no se me ocurre
	float dt = static_cast<float>(t) * gTimeScale;

	//para la explosion
	if (gExpl) gExpl->advance(dt);
		
	if (gForceReg) gForceReg->updateForces(dt);


	if (gMode == Mode::Projectiles) {
		for (auto* p : gProjectiles) p->integrate(dt);
		//limpieza de cositas
		gWorld.removeOutside(gProjectiles);
	}
		// Emitters
	else {
		if (gEmitSys) gEmitSys->update(dt);
		
	}
		

	
}

// Function to clean data
// Add custom code to the begining of the function
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);

	//LIMPIEZA PRINCIPAL
	ClearProjectiles();
	ClearEmitters();

	// ======== PRACTICA 0: DEREGISTRAR ========
	if (gOriginSphere) { DeregisterRenderItem(gOriginSphere); gOriginSphere = nullptr; }
	if (gAxisX) { DeregisterRenderItem(gAxisX);        gAxisX = nullptr; }
	if (gAxisY) { DeregisterRenderItem(gAxisY);        gAxisY = nullptr; }
	if (gAxisZ) { DeregisterRenderItem(gAxisZ);        gAxisZ = nullptr; }
	// ======== FIN PRACTICA 0 ========	

	// Emitters (objetos)a
	delete gEmit1; gEmit1 = nullptr;
	delete gEmit2; gEmit2 = nullptr;
	delete gEmit3; gEmit3 = nullptr;

	//--fuerzas--
	if (gGravity) { delete gGravity;  gGravity = nullptr; }
	if (gForceReg) { delete gForceReg; gForceReg = nullptr; }
	if (gWind) { delete gWind; gWind = nullptr; }
	if (gWhirl) { delete gWhirl; gWhirl = nullptr; }
	if (gExpl) { delete gExpl; gExpl = nullptr; }




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
	//modos (proyectil y emisores)
case 'P': SetMode(Mode::Projectiles); break;
case 'E': SetMode(Mode::Emitters);    break;

case '+': //letnitud
	gTimeScale = std::min(10.0f, gTimeScale * 1.5f);
	display_text = "TimeScale x" + std::to_string(gTimeScale);
	break;
case '-': //rapidez
	gTimeScale = std::max(0.1f, gTimeScale / 1.5f);
	display_text = "TimeScale x" + std::to_string(gTimeScale);
	break;

case 'V': { //interruptor de viento
	static bool windOn = true;           
	windOn = !windOn;
	if (gWind) gWind->setK1(windOn ? 2.0f : 0.0f);
	display_text = windOn ? "Wind: ON (k1=2)" : "Wind: OFF";
	break;
}
case 'X': { //hace el viento más fuerte
	if (gWind) {
		gWind->setWind(Vector3D(12.0f, 0.0f, 0.0f));
		//por si se apaga el viento
		gWind->setK1(2.0f);
	}
	display_text = "Wind: +X strong";
	break;
}
case 'O': { // interruptor torbellino ON/OFF alterando k1
	static bool whirlOn = true;
	whirlOn = !whirlOn;
	if (gWhirl) gWhirl->setK1(whirlOn ? 2.0f : 0.0f);
	display_text = whirlOn ? "Whirlwind: ON" : "Whirlwind: OFF";
	break;
}

case 'G': { //interruptor gravedad
	if (!gGravity) break;
	const Vector3D& cur = gGravity->getG();
	const bool on = (cur.x != 0.0f) || (cur.y != 0.0f) || (cur.z != 0.0f);
	gGravity->setG(on ? Vector3D(0.0f, 0.0f, 0.0f)
		: Vector3D(0.0f, -9.8f, 0.0f));
	display_text = on ? "Gravity: OFF" : "Gravity: ON";
	break;
}

case 'M': { //explosión en origen de coordenadas	
	TriggerExplosionAt(Vector3D(0.0f, 0.0f, 0.0f));  // origen
	display_text = "Explosion @ (0,0,0)";
	break;
}

case 'N': //explosion custom
	TriggerExplosionAt(Vector3D(2.0f, 4.0f, 0.0f)); //custom
	display_text = "Explosion @ (2,4,0)";
	break;

case 'L': case 'l': { //debug para saber cuantos FG afectan a la ultima particula
	if (gProjectiles.empty()) break;
	Particle* p = gProjectiles.back();
	int nG = 0, nW = 0, nWh = 0;
	for (auto& r : gForceReg->regs) { 
		if (r.p != p) continue;
		if (r.fg == gGravity)   ++nG;
		if (r.fg == gWind)      ++nW;
		if (r.fg == gWhirl) ++nWh;
	}
	printf("[DBG] last projectile -> Grav:%d Wind:%d Whirl:%d\n", nG, nW, nWh);
	break;
}



default:
	if (gMode == Mode::Projectiles) {
		switch (toupper(key)) { //aseguramos con v=0 para que se por fuerza (minidebug)
		case 'B': SpawnProjectile(0, 0.991f, { 1.0f,0.8f,0.2f,1.0f }, 0.15f,1.0f); break; // bala (amarillo)
		case 'Z': SpawnProjectile(0, 0.995f, { 1.0f,1.0f,1.0f,1.0f }, 0.20f,3.0f); break; // pelota (blanco)
		case 'H': SpawnProjectile(0, 0.991f, { 0.7f,0.9f,1.0f,1.0f }, 0.8f,0.3f); break; // helio (azulado)
		case 'C': ClearProjectiles(); display_text = "Projectiles: clear"; break;
		default: break;
		}
	}
	else {
		//emisores 1 y 2 normales 3 gauusiano
		switch (toupper(key)) {
		case '1': gEmit1->setActive(!gEmit1->isActive()); break;
		case '2': gEmit2->setActive(!gEmit2->isActive()); break;
		case '3': gEmit3->setActive(!gEmit3->isActive()); break;
		case '[': if (gEmit1->isActive()) gEmit1->changeRate(-1.0f);
			if (gEmit2->isActive()) gEmit2->changeRate(-1.0f);
			if (gEmit3->isActive()) gEmit3->changeRate(-1.0f);
			display_text = "Emitters: rate -"; break;
		case ']': if (gEmit1->isActive()) gEmit1->changeRate(+1.0f);
			if (gEmit2->isActive()) gEmit2->changeRate(+1.0f);
			if (gEmit3->isActive()) gEmit3->changeRate(+1.0f);
			display_text = "Emitters: rate +"; break;
		case 'C': ClearEmitters();  display_text = "Emitters: clear"; break;
		default: break;
		}
	}
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