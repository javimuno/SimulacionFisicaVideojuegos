#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <ctype.h>
#include <PxPhysicsAPI.h>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <Windows.h>

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
#include "Render/Camera.h"



using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation* gFoundation = NULL;
PxPhysics* gPhysics = NULL;


PxMaterial* gMaterial = NULL;

PxPvd* gPvd = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene* gScene = NULL;
ContactReportCallback gContactReportCallback;

//=========== GLOBALES =========


// RenderItems de la Práctica 0
RenderItem* gOriginSphere = nullptr;
RenderItem* gAxisX = nullptr;
RenderItem* gAxisY = nullptr;
RenderItem* gAxisZ = nullptr;


// === Constantes para angulos===
static constexpr float PI = 3.14159265358979323846f;
static constexpr float DEG2RAD = PI / 180.0f;

//=== Del juego ===
static Particle* gPlayerVis[2] = { nullptr, nullptr };
static std::vector<Particle*> gPlatforms;

// === HUD ===
std::string display_text = "P:Projectiles  E:Emitters";

// === Emitters (deco) + vector de proyectiles ===
static ParticleSystem* gEmitSys = nullptr;
static SimpleEmitter* gEmit1 = nullptr;
static SimpleEmitter* gEmit2 = nullptr;
static SimpleEmitter* gEmit3 = nullptr;

// Lista de proyectiles del jugador 
static std::vector<Particle*> gProjectiles;


// Crea los límites una vez
WorldBounds gWorld(Vector3D(-50, -50, -50), Vector3D(50, 50, 50));

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
			if (timeLeft_ <= 0.0f) return;   // 
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
enum class Mode { Projectiles, Emitters, Theater };
static Mode gMode = Mode::Theater;

//para coger la endemoniada camara de una p**-** vez
static void SnapCameraToXY()
{
	auto* cam = GetCamera();          
	if (!cam) return;

	physx::PxVec3 eye(0.f, 7.0f, 22.0f);
	physx::PxVec3 at(0.f, 7.0f, 0.0f);
	physx::PxVec3 dir = (at - eye).getNormalized();

	
	*cam = Snippets::Camera(eye, dir);
}



												/*JUEGO-TEATRO ANTARTICA*/








struct GameState {
	Vector3D pos[2];           // posiciones jugadores
	int      current = 0;      // 0/1
	float    angleDeg[2] = { 45.f, 45.f };
	float    power01 = 0.f;
	float    chargeTimeSec = 0.f;
	bool     charging = false;
	bool     shotInFlight = false;
	int      projType = 0;     // 0=B,1=Z,2=H
	int      score[2] = { 0,0 };
	Vector3D wind = { 0,0,0 };   // viento de juego
	int      round = 0;
} gGame;

// --- fuerzas SOLO de juego (proyectiles) ---
GravityFG* gGravGame = nullptr;
WindFG* gWindGame = nullptr;

// --- límites del escenario (cuadrado(Teatro)) ---
static constexpr float STAGE_L = 20.0f;
static constexpr float STAGE_HH = STAGE_L * 0.5f;

struct StageBounds {
	float minX = -STAGE_HH, maxX = STAGE_HH;
	float minY = 0.0f, maxY = STAGE_L;  // suelo en y=0
} gStage;

// --- UTILES ---
static inline float randRange(float a, float b) {
	return a + (b - a) * (float)rand() / (float)RAND_MAX;
}

static inline void clamp2D(Particle* p) {
	auto v = p->getVelocity(); v.z = 0; p->setVelocity(v);
	auto x = p->getPosition(); x.z = 0; p->setPosition(x);
}

static inline bool outOfStage2D(const Vector3D& x) {
	return (x.x < gStage.minX || x.x > gStage.maxX || x.y < gStage.minY || x.y > gStage.maxY);
}


static void StartRound() {
	gGame.round++;

	// plataformas fijas
	static const Vector3D A[] = { {-9.f, 2.0f, 0.f}, {-9.f, 3.2f, 0.f}, {-9.f, 4.4f, 0.f} };
	static const Vector3D B[] = { {+9.f, 2.6f, 0.f}, {+9.f, 3.8f, 0.f}, {+9.f, 5.0f, 0.f} };
	int i = gGame.round % 3;
	gGame.pos[0] = A[i];
	gGame.pos[1] = B[(i + 1) % 3];

	// viento SOLO del juego (ni modo E ni P)
	float wx = randRange(-1.0f, +1.0f);
	gGame.wind = { wx, 0.f, 0.f };
	if (gWindGame) gWindGame->setWind(gGame.wind);

	// reset input
	gGame.power01 = 0.f;
	gGame.chargeTimeSec = 0.f;
	gGame.charging = false;
	gGame.shotInFlight = false;
}

//jugadores y paltaformas
static Particle* MakeStaticBall(const Vector3D& pos, float radius, const physx::PxVec4& col)
{
	Vector3D v0{ 0,0,0 }, a0{ 0,0,0 };
	// damping 1, masa 1
	return new Particle(pos, v0, a0, 1.0f, IntegratorType::EulerSemiImplicit, 1.0f, col, radius);
}

// borra todas las plataformas
static void ClearPlatforms()
{
	for (auto* p : gPlatforms) delete p;
	gPlatforms.clear();
}

// crea una "barra" de plataforma con esferas pequeñas
static void MakePlatformBar(const Vector3D& p0, float halfLen, float y, const physx::PxVec4& col)
{
	const int N = 8;
	for (int i = 0; i < N; ++i) {
		float t = (i / (float)(N - 1)) * 2.f - 1.f; // [-1,1]
		Vector3D p = { p0.x + t * halfLen, y, 0.f };
		gPlatforms.push_back(MakeStaticBall(p, /*r*/0.15f, col));
	}
}



static void SpawnProjectileFromPlayer(const Vector3D& pos,
	const Vector3D& dirIn,
	float speed,
	int   projType)
{
	// dirección unitaria
	Vector3D dir = dirIn;
	float n = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
	if (n > 1e-6f) { dir.x /= n; dir.y /= n; dir.z /= n; }
	else { dir = { 1,0,0 }; }

	// propiedades básicas por tipo de proyectil
	float mass = 1.0f, damp = 0.995f, radius = 0.2f;
	physx::PxVec4 color(1, 1, 1, 1);

	switch (projType) {
	case 0: mass = 1.0f; damp = 0.990f; radius = 0.35f; color = physx::PxVec4(1.f, 0.85f, 0.2f, 1.f); break; // bala
	case 1: mass = 1.5f; damp = 0.995f; radius = 0.80f; color = physx::PxVec4(0.2f, 0.2f, 0.2f, 1.f);   break; // canon
	case 2: mass = 0.3f; damp = 0.995f; radius = 0.30f; color = physx::PxVec4(0.7f, 0.9f, 1.f, 1.f); break; // helio
	}

	// nace en reposo
	Vector3D v0{ 0,0,0 }, a0{ 0,0,0 };
	auto* p = new Projectile(pos, v0, a0, damp,
		IntegratorType::EulerSemiImplicit,
		mass, color, radius, ProjectileType::Light);

	// fuerzas SOLO de juego en THEATER; en otros modos, las globales
	if (gMode == Mode::Theater) {
		if (gForceReg && gGravGame) gForceReg->add(p, gGravGame);
		if (gForceReg && gWindGame) gForceReg->add(p, gWindGame);
	}
	else {
		if (gForceReg && gGravity) gForceReg->add(p, gGravity);
		if (gForceReg && gWind)    gForceReg->add(p, gWind);
		if (gForceReg && gWhirl)   gForceReg->add(p, gWhirl);
		if (gForceReg && gExpl)    gForceReg->add(p, gExpl);
	}

	// impulso de fueraza F (constante en N=loquesea o asi para que la barra de espacio cargue)
	static constexpr float kImpulseT = 0.15f;
	float m = p->getMass();               
	float N = m * (speed / kImpulseT);
	auto* kick = new TimedKickFG(dir, N, kImpulseT);
	if (gForceReg) gForceReg->add(p, kick);

	gProjectiles.push_back(p);
	clamp2D(p);
}

// mecanismo: dispara el jugador actual con su ángulo y potencia
static void FireCurrentPlayer() {
	float theta = gGame.angleDeg[gGame.current] * DEG2RAD;
	float sgn = (gGame.current == 0) ? +1.f : -1.f;      // J1 mira hacia +X, J2 mira hacia -X
	Vector3D dir(std::cos(theta) * sgn, std::sin(theta), 0.f);

	float baseMin = 12.f, baseMax = 28.f;
	float speed = baseMin + gGame.power01 * (baseMax - baseMin);

	SpawnProjectileFromPlayer(gGame.pos[gGame.current], dir, speed, gGame.projType);
	gGame.shotInFlight = true;
}


static bool CheckHitAndOut() {
	if (gProjectiles.empty()) return false;
	Particle* p = gProjectiles.back();
	Vector3D x = p->getPosition();

	// objetivo = rival
	int enemy = gGame.current ^ 1;
	Vector3D t = gGame.pos[enemy];

	float dx = x.x - t.x, dy = x.y - t.y;
	float dist = std::sqrt(dx * dx + dy * dy);
	const float hitR = 0.75f;

	if (dist <= hitR) {
		// en un futuro explosion (todavia solo manual)

		gGame.score[gGame.current]++;
		gGame.shotInFlight = false;
		return true;
	}
	if (outOfStage2D(x)) {
		gGame.shotInFlight = false;
		return true;
	}
	return false;
}


static void EnterTheaterMode() {

	//sin origenes
	if (gOriginSphere) { DeregisterRenderItem(gOriginSphere); gOriginSphere = nullptr; }
	if (gAxisX) { DeregisterRenderItem(gAxisX); gAxisX = nullptr; }
	if (gAxisY) { DeregisterRenderItem(gAxisY); gAxisY = nullptr; }
	if (gAxisZ) { DeregisterRenderItem(gAxisZ); gAxisZ = nullptr; }


	// fuerzas del juego juego
	delete gGravGame;  gGravGame = new GravityFG(Vector3D(0.f, -9.81f, 0.f));
	delete gWindGame;  gWindGame = new WindFG(Vector3D(0.f, 0.f, 0.f), 0.9f, 0.02f);

	// decoración: los emisores que ya teniamos
	if (gGravity) delete gGravity;
	if (gWind)    delete gWind;
	if (gWhirl)   delete gWhirl;

	gGravity = new GravityFG(Vector3D(0.f, -1.8f, 0.f));                 // nieve
	gWind = new WindFG(Vector3D(0.f, 0.f, 0.f), 0.1f, 0.0f);      // brisa
	ZoneSphere z(Vector3D(0.f, 10.f, 0.f), 4.0f);
	gWhirl = new WhirlwindFG(z, 2.5f, 1.0f, 0.6f, 0.4f, 0.0f);         // decorativo

	// configurar emisores 1–3 como deco
	if (gEmit1) {
		SimpleEmitterConfig c{};
		c.position = { 0.f, gStage.maxY - 0.3f, -1.0f };
		c.posJitter = { gStage.maxX * 0.9f, 0.6f, 1.2f };
		c.speed = 3.0f;
		c.lifetime = 10.0f;
		c.damping = 0.998f;
		c.color = physx::PxVec4(1, 1, 1, 1);
		c.mass = 0.2f;
		c.radius = 0.05f;
		c.rate = 50.f;
		c.maxAlive = 600;

		gEmit1->clear();
		gEmit1->setConfig(c);
		gEmit1->setActive(true);
	}
	if (gEmit2) {
		auto c = gEmit1->getConfig();
		c.position = { -5.f, gStage.maxY - 0.8f, +1.0f };
		gEmit2->clear();
		gEmit2->setConfig(c);
		gEmit2->setActive(true);
	}
	if (gEmit3) {
		auto c = gEmit1->getConfig();
		c.position = { +5.f, gStage.maxY - 1.0f, +1.5f };
		gEmit3->clear();
		gEmit3->setConfig(c);
		gEmit3->setActive(true);
		
	}

	
	StartRound();


	// Jugadores visibles (azul y rojo) 
	if (gPlayerVis[0]) { delete gPlayerVis[0]; gPlayerVis[0] = nullptr; }
	if (gPlayerVis[1]) { delete gPlayerVis[1]; gPlayerVis[1] = nullptr; }
	gPlayerVis[0] = MakeStaticBall(gGame.pos[0], 0.40f, physx::PxVec4(0.2f, 0.5f, 1.f, 1.f)); // azul
	gPlayerVis[1] = MakeStaticBall(gGame.pos[1], 0.40f, physx::PxVec4(1.f, 0.2f, 0.2f, 1.f));  // rojo

	// Plataformas (una barra bajo cada jugador)
	ClearPlatforms();
	MakePlatformBar(gGame.pos[0], 1.8f, gGame.pos[0].y - 0.35f, physx::PxVec4(0.8f, 0.8f, 0.8f, 1.f));
	MakePlatformBar(gGame.pos[1], 1.8f, gGame.pos[1].y - 0.35f, physx::PxVec4(0.8f, 0.8f, 0.8f, 1.f));


	SnapCameraToXY();
}

static void ExitTheaterMode() {
	// limpiar proyectiles del juego
	for (auto* p : gProjectiles) delete p;
	gProjectiles.clear();
	//borrar jugadores
	if (gPlayerVis[0]) { delete gPlayerVis[0]; gPlayerVis[0] = nullptr; }
	if (gPlayerVis[1]) { delete gPlayerVis[1]; gPlayerVis[1] = nullptr; }

	ClearPlatforms();

	// eliminamos los generadores de F del juego
	delete gGravGame; gGravGame = nullptr;
	delete gWindGame; gWindGame = nullptr;
}

											/*JUEGO-TEATRO ANTARTICA*/




// escala de tiempo ---->> de momento esto no 
float gTimeScale = 1.0f; //multiplicador de dt en la integración
// --- estado de juego ---



//METODOS

static void ClearProjectiles() {
	for (auto* p : gProjectiles) {
		if (gForceReg) {
			if (gGravity) gForceReg->remove(p, gGravity);
			if (gWind)    gForceReg->remove(p, gWind);
			if (gWhirl)   gForceReg->remove(p, gWhirl);
			if (gExpl)    gForceReg->remove(p, gExpl);
			//juego
			if (gGravGame) gForceReg->remove(p, gGravGame);
			if (gWindGame) gForceReg->remove(p, gWindGame);
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

static void SetMode(Mode m)
{
	if (gMode == m) return;

	// 
	switch (gMode) {
	case Mode::Theater:   ExitTheaterMode(); break;
	case Mode::Projectiles:
	case Mode::Emitters:   break;
	}

	// entrar al nuevo modo
	switch (m) {
	case Mode::Projectiles:
		ClearEmitters();
		ClearProjectiles();
		
		display_text = "Projectiles: B/Z/H, C  |  +/- time";
		break;

	case Mode::Emitters:
		ClearProjectiles();
		
		display_text = "Emitters: 1/2/3 toggle, [ ] rate, C clear  |  +/- time";
		break;

	case Mode::Theater:
		ClearProjectiles();
		ClearEmitters();
		EnterTheaterMode();
		display_text = "THEATER 2D: angulo A/D, potencia Espacio, 1/2/3 tipo";
		break;
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

	//setup de fuerzas-> IMPORTANTE SE PONE LA FUERZA AQUI 
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
		/*omega*/   3.5f, // giro tangencial
		/*updraft*/ 15.0f, //elevacion
		/*radialIn*/1.0f, // fuerza de succion al centro si es 0 no hay o esta en el centro
		/*k1*/      10.0f,  // (lineal)- > laminar
		/*k2*/      0.0f);  //mas fuerte cuando mayor es vRel (turbulento)

	//Explosion
	gExpl = new ExplosionFG(Vector3D(0, 0, 0), /*R*/60.0f, /*K*/800.0f, /*tau*/80.8f);






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
	
	//===========EMISORES=======================

	{
		SimpleEmitterConfig c1; // Azul: fuente suave
		c1.position = { 2,0,2 };
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

		SimpleEmitterConfig c3; // Amarillo: chispas y gauss
		c3.position = { 0,0,0 };
		c3.posJitter = { 10.f,0.f,0.f }; //jitter en x es Sigma
		c3.speed = 1.0f;
		c3.lifetime = 6.6f;
		c3.damping = 0.995f;
		c3.color = { 1.0f,0.8f,0.2f,1.0f };
		c3.mass = 1.1f;
		c3.radius = 0.37f;
		c3.rate = 8.0f;
		c3.maxAlive = 180;
		c3.active = false;
		gEmit3 = new SimpleEmitter(c3);
		gEmit3->setGaussianCone(true);
		auto cfg3 = gEmit3->getConfig();
		cfg3.posGaussX = true;
		gEmit3->setConfig(cfg3);
		
	}

	gEmitSys = new ParticleSystem(&gWorld);
	gEmitSys->addEmitter(gEmit1);
	gEmitSys->addEmitter(gEmit2);	
	gEmitSys->addEmitter(gEmit3);
	

	SetMode(Mode::Theater);

	
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

	switch (gMode) {
	case Mode::Projectiles: {
		
		for (auto it = gProjectiles.begin(); it != gProjectiles.end(); ) {
			Particle* p = *it;
			p->integrate(dt);
			if (gWorld.isOutside(p->getPosition())) { delete p; it = gProjectiles.erase(it); }
			else ++it;
		}
		break;
	}
	case Mode::Emitters: {
		if (gEmitSys) gEmitSys->update(dt);
		break;
	}
	case Mode::Theater:
	{
		// --- INPUT espacio: mantener para cargar, soltar para disparar ---
		static bool prevSpaceDown = false;
		bool spaceDown = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;

		if (spaceDown && !prevSpaceDown && !gGame.shotInFlight) {
			// carga
			gGame.charging = true;
			gGame.chargeTimeSec = 0.f;
			gGame.power01 = 0.f;
		}
		if (!spaceDown && prevSpaceDown && gGame.charging) {
			// disparo
			gGame.charging = false;
			FireCurrentPlayer();
		}
		prevSpaceDown = spaceDown;

		// Carga progresiva 
		if (gGame.charging && !gGame.shotInFlight) {
			gGame.chargeTimeSec += dt;
			const float T = 2.0f; // carga completa en 2s
			gGame.power01 = (gGame.chargeTimeSec < T) ? (gGame.chargeTimeSec / T) : 1.0f;
		}

		// Proyectiles del juego: integrar, fijar z=0 y limpiar si salen de los limites del juego
		for (auto it = gProjectiles.begin(); it != gProjectiles.end();) {
			Particle* p = *it;
			p->integrate(dt);
			clamp2D(p);
			if (outOfStage2D(p->getPosition())) {
				delete p; it = gProjectiles.erase(it);
				gGame.shotInFlight = false;
			}
			else ++it;
		}

		// No deja disparar si no ha desaparecido el proyectil
		if (gGame.shotInFlight) {

			//si has golpeado al jugador enemigo
			if (CheckHitAndOut()) {
				gGame.current ^= 1;   // cambio de jugador
				StartRound();         // pone viento nuevo y resetea inputs

				//cambia jugadores
				if (gPlayerVis[0]) { delete gPlayerVis[0]; gPlayerVis[0] = nullptr; }
				if (gPlayerVis[1]) { delete gPlayerVis[1]; gPlayerVis[1] = nullptr; }
				gPlayerVis[0] = MakeStaticBall(gGame.pos[0], 0.40f, physx::PxVec4(0.2f, 0.5f, 1.f, 1.f));
				gPlayerVis[1] = MakeStaticBall(gGame.pos[1], 0.40f, physx::PxVec4(1.f, 0.2f, 0.2f, 1.f));

				// Y recrea las plataformas
				ClearPlatforms();
				MakePlatformBar(gGame.pos[0], 1.8f, gGame.pos[0].y - 0.35f, physx::PxVec4(0.8f, 0.8f, 0.8f, 1.f));
				MakePlatformBar(gGame.pos[1], 1.8f, gGame.pos[1].y - 0.35f, physx::PxVec4(0.8f, 0.8f, 0.8f, 1.f));
			}
		}

		// Decoración (nieve/torbellinos)
		if (gEmitSys) gEmitSys->update(dt);

		// HUD del modo THEATER
		const char* names[] = { "Bala", "Canon", "Helio" };
		char buf[256];
		snprintf(buf, sizeof(buf),
			"THEATER 2D | Turno: J%d  |  Proyectil: %s  |  Angulo: %.1f  |  Potencia: %d%%  |  VientoX: %.1f m/s  |  %d - %d",
			gGame.current + 1, names[gGame.projType],
			gGame.angleDeg[gGame.current],
			(int)(gGame.power01 * 100.f),
			gGame.wind.x,
			gGame.score[0], gGame.score[1]);
		display_text = buf;
		break;

		// sincroniza solo por si StartRound cambia posiciones
		if (gPlayerVis[0]) gPlayerVis[0]->setPosition(gGame.pos[0]);
		if (gPlayerVis[1]) gPlayerVis[1]->setPosition(gGame.pos[1]);
	}
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

    // Cambios de modo y time scale (comunes)
    switch (toupper(key)) {
    case 'P': SetMode(Mode::Projectiles); return;
    case 'E': SetMode(Mode::Emitters);    return;
    case 'T': SetMode(Mode::Theater); SnapCameraToXY();    return;

    case '+':
        gTimeScale = std::min(10.0f, gTimeScale * 1.5f);
        display_text = "TimeScale x" + std::to_string(gTimeScale);
        return;
    case '-':
        gTimeScale = std::max(0.1f, gTimeScale / 1.5f);
        display_text = "TimeScale x" + std::to_string(gTimeScale);
        return;
    default: break;
    }

    // ----- CONTROLES POR MODO -----

    // 1) THEATER (2D, tiro por fuerzas)
    if (gMode == Mode::Theater) {
        // Tipo de proyectil
        if (key == '1') { gGame.projType = 0; return; }
        if (key == '2') { gGame.projType = 1; return; }
        if (key == '3') { gGame.projType = 2; return; }

        // Ángulo (no usamos WASD)
        const float stepDeg = 1.5f;
        switch (toupper(key)) {
        case 'J':
            gGame.angleDeg[gGame.current] -= stepDeg;
            if (gGame.angleDeg[gGame.current] < 10.f) gGame.angleDeg[gGame.current] = 10.f;
            return;
        case 'L':
            gGame.angleDeg[gGame.current] += stepDeg;
            if (gGame.angleDeg[gGame.current] > 80.f) gGame.angleDeg[gGame.current] = 80.f;
            return;
        case 'R': // reiniciar ronda (útil para pruebas)
            if (!gGame.shotInFlight) StartRound();
            return;
		case 'V': { //interruptor de viento
			static bool windOn = true;
			windOn = !windOn;
			if (gWind) gWind->setK1(windOn ? 2.0f : 0.0f);
			display_text = windOn ? "Wind: ON (k1=2)" : "Wind: OFF";
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

		case 'M': { //explosin en origen de coordenadas	
			TriggerExplosionAt(Vector3D(0.0f, 0.0f, 0.0f));  // origen
			display_text = "Explosion @ (0,0,0)";
			break;
		}

		default: break;
		}

     

        // Limpiar proyectiles del gameplay
        if (toupper(key) == 'C') {
            ClearProjectiles();
            gGame.shotInFlight = false;
            return;
        }

        return; // fin THEATER
    }

    // 2) PROJECTILES (tu modo de pruebas de proyectiles sueltos)
    if (gMode == Mode::Projectiles) {
        switch (toupper(key)) {
        case 'B': SpawnProjectile(20.0f, 0.91f,  { 1.0f,0.8f,0.2f,1.0f }, 0.15f,1.0f); break; // bala (amarillo)
        case 'Z': SpawnProjectile(20.0f, 0.995f, { 1.0f,1.0f,1.0f,1.0f }, 0.20f,3.0f); break; // pelota (blanco)
        case 'H': SpawnProjectile(20.0f, 1.0f,   { 0.7f,0.9f,1.0f,1.0f }, 0.80f,0.3f); break; // helio (azulado)
        case 'C': ClearProjectiles(); display_text = "Projectiles: clear"; break;
		case 'V': { //interruptor de viento
			static bool windOn = true;
			windOn = !windOn;
			if (gWind) gWind->setK1(windOn ? 2.0f : 0.0f);
			display_text = windOn ? "Wind: ON (k1=2)" : "Wind: OFF";
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

		case 'M': { //explosin en origen de coordenadas	
			TriggerExplosionAt(Vector3D(0.0f, 0.0f, 0.0f));  // origen
			display_text = "Explosion @ (0,0,0)";
			break;
		}

		
        default: break;
        }
        return;
    }

    // 3) EMITTERS (tu modo de emisores de partículas)
    // Teclas: 1/2/3 activan, [ ] ajustan rate, C limpia vivos
    if (gMode == Mode::Emitters) {
        switch (toupper(key)) {
        case '1': if (gEmit1) gEmit1->setActive(!gEmit1->isActive()); break;
        case '2': if (gEmit2) gEmit2->setActive(!gEmit2->isActive()); break;
        case '3': if (gEmit3) gEmit3->setActive(!gEmit3->isActive()); break;

        case '[':
            if (gEmit1 && gEmit1->isActive()) gEmit1->changeRate(-1.0f);
            if (gEmit2 && gEmit2->isActive()) gEmit2->changeRate(-1.0f);
            if (gEmit3 && gEmit3->isActive()) gEmit3->changeRate(-1.0f);
            display_text = "Emitters: rate -";
            break;

        case ']':
            if (gEmit1 && gEmit1->isActive()) gEmit1->changeRate(+1.0f);
            if (gEmit2 && gEmit2->isActive()) gEmit2->changeRate(+1.0f);
            if (gEmit3 && gEmit3->isActive()) gEmit3->changeRate(+1.0f);
            display_text = "Emitters: rate +";
            break;

        case 'C':
			ClearEmitters();
            display_text = "Emitters: clear";
            break;

		case 'V': { //interruptor de viento
			static bool windOn = true;
			windOn = !windOn;
			if (gWind) gWind->setK1(windOn ? 2.0f : 0.0f);
			display_text = windOn ? "Wind: ON (k1=2)" : "Wind: OFF";
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

		case 'M': { //explosin en origen de coordenadas	
			TriggerExplosionAt(Vector3D(0.0f, 0.0f, 0.0f));  // origen
			display_text = "Explosion @ (0,0,0)";
			break;
		}

		case 'H':
			if (gEmit3) {
				std::cout << "\n=== HISTO Emisor3 (X) ===\n";
				gEmit3->debugPrintHistogramX(25, 20000, 3.0f); // 25 bins, 20k muestras
				display_text = "Emitters: printed histogram to console";
			}
			return;

        default: break;
        }
        return;
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