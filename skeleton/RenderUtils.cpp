#include <vector>

#include "PxPhysicsAPI.h"

#include "core.hpp"
#include "RenderUtils.hpp"
#include "Vector3D.h"


using namespace physx;

extern void initPhysics(bool interactive);
extern void stepPhysics(bool interactive, double t);	
extern void cleanupPhysics(bool interactive);
extern void keyPress(unsigned char key, const PxTransform& camera);
extern PxPhysics* gPhysics;
extern PxMaterial* gMaterial;
extern PxScene* gScene; // para la escenafisica

std::vector<const RenderItem*> gRenderItems;

double PCFreq = 0.0;
__int64 CounterStart = 0;
__int64 CounterLast = 0;

////paradibujar los ejes
//
//void drawCoordinateAxes() {
//	// Crear las geometr�as de las l�neas de los ejes
//	PxBoxGeometry axisGeometry(PxVec3(0.1f, 0.1f, 10.0f)); // Ejes con longitud 20 (extensi�n en Z)
//
//	// Eje X (rojo)
//	PxRigidStatic* xAxisActor = gPhysics->createRigidStatic(PxTransform(PxVec3(10, 0, 0)));  // Posici�n en el espacio
//	PxShape* xAxisShape = CreateShape(axisGeometry, gMaterial);
//	xAxisActor->attachShape(*xAxisShape);
//	gScene->addActor(*xAxisActor);
//	RenderItem* xAxisRenderItem = new RenderItem(xAxisShape, xAxisActor, PxVec4(1.0f, 0.0f, 0.0f, 1.0f)); // Color rojo
//	RegisterRenderItem(xAxisRenderItem);
//
//	// Eje Y (verde)
//	PxRigidStatic* yAxisActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 10, 0)));
//	PxShape* yAxisShape = CreateShape(axisGeometry, gMaterial);
//	yAxisActor->attachShape(*yAxisShape);
//	gScene->addActor(*yAxisActor);
//	RenderItem* yAxisRenderItem = new RenderItem(yAxisShape, yAxisActor, PxVec4(0.0f, 1.0f, 0.0f, 1.0f)); // Color verde
//	RegisterRenderItem(yAxisRenderItem);
//
//	// Eje Z (azul)
//	PxRigidStatic* zAxisActor = gPhysics->createRigidStatic(PxTransform(PxVec3(0, 0, 10)));
//	PxShape* zAxisShape = CreateShape(axisGeometry, gMaterial);
//	zAxisActor->attachShape(*zAxisShape);
//	gScene->addActor(*zAxisActor);
//	RenderItem* zAxisRenderItem = new RenderItem(zAxisShape, zAxisActor, PxVec4(0.0f, 0.0f, 1.0f, 1.0f)); // Color azul
//	RegisterRenderItem(zAxisRenderItem);
//}


void StartCounter()
{
	LARGE_INTEGER li;
	if (!QueryPerformanceFrequency(&li))
		return;

	PCFreq = double(li.QuadPart) /*/ 1000.0*/;

	QueryPerformanceCounter(&li);
	CounterStart = li.QuadPart;
	CounterLast = CounterStart;
}

double GetCounter()
{
	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	double t = double(li.QuadPart - CounterLast) / PCFreq;
	CounterLast = li.QuadPart;
	return t;
}

namespace
{
	Camera*	sCamera;

void motionCallback(int x, int y)
{
	sCamera->handleMotion(x, y);
}

void keyboardCallback(unsigned char key, int x, int y)
{
	if(key==27)
		exit(0);

	if(!sCamera->handleKey(key, x, y))
		keyPress(key, sCamera->getTransform());
}

void mouseCallback(int button, int state, int x, int y)
{
	sCamera->handleMouse(button, state, x, y);
}

void idleCallback()
{
	glutPostRedisplay();
}

float stepTime = 0.0f;
//#define FIXED_STEP

void renderCallback()
{
	double t = GetCounter();
#ifdef FIXED_STEP
	if (t < (1.0f / 30.0f))
	{
		fprintf(stderr, "Time: %f\n", stepTime);
		stepTime += t;
	}
	else
		stepTime = 1.0f / 30.0f;

	if (stepTime >= (1.0f / 30.0f))
	{
		stepPhysics(true, stepTime);
		stepTime = 0.0f;
	}
#else
	stepPhysics(true, t);
#endif

	startRender(sCamera->getEye(), sCamera->getDir());

	//fprintf(stderr, "Num Render Items: %d\n", static_cast<int>(gRenderItems.size()));
	for (auto it = gRenderItems.begin(); it != gRenderItems.end(); ++it)
	{
		const RenderItem* obj = (*it);
		auto objTransform = obj->transform;
		if (!objTransform)
		{
			auto actor = obj->actor;
			if (actor)
			{
				renderShape(*obj->shape, actor->getGlobalPose(), obj->color);
				continue;
			}
		}
		renderShape(*obj->shape, objTransform ? *objTransform : physx::PxTransform(PxIdentity), obj->color);
	}

	//PxScene* scene;
	//PxGetPhysics().getScenes(&scene, 1);
	//PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
	//if (nbActors)
	//{
	//	std::vector<PxRigidActor*> actors(nbActors);
	//	scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
	//	renderActors(&actors[0], static_cast<PxU32>(actors.size()), true, Vector4(1.0f, 0.0f, 0.0f, 1.0f));
	//}

	finishRender();
}

void exitCallback(void)
{
	delete sCamera;
	cleanupPhysics(true);
}
}

void renderLoop()
{
	StartCounter();
	sCamera = new Camera(PxVec3(50.0f, 50.0f, 50.0f), PxVec3(-0.6f,-0.2f,-0.7f));

	setupDefaultWindow("Simulacion Fisica Videojuegos");
	setupDefaultRenderState();

	glutIdleFunc(idleCallback);
	glutDisplayFunc(renderCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	motionCallback(0,0);

	atexit(exitCallback);

	initPhysics(true);
	glutMainLoop();
}

void RegisterRenderItem(const RenderItem* _item)
{
	gRenderItems.push_back(_item);
}

void DeregisterRenderItem(const RenderItem* _item)
{
	auto it = find(gRenderItems.begin(), gRenderItems.end(), _item);
	gRenderItems.erase(it);
}

double GetLastTime()
{
	double t = double(CounterLast - CounterStart) / PCFreq;
	return t;
}

Camera* GetCamera()
{
	return sCamera;
}

PxShape* CreateShape(const PxGeometry& geo, const PxMaterial* mat)
{
	if (mat == nullptr)
		mat = gMaterial; // Default material

	PxShape* shape = gPhysics->createShape(geo, *mat);
	return shape;
}


