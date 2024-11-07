#pragma once

#include "Vector3D.h"
#include "RenderUtils.hpp"
#include "core.hpp"

class Particle
{

public:


	Vector3D vel;
	Vector3D acc; // (3) aceleracion de particula
	Vector3D prevPos; // (4) pos anterior para Verlet
	bool op;
	float damping; //(3) damping
	physx::PxTransform pose; //a render item le pasamos la direccion de esta pose
	//para que se actualice automaticamente
	RenderItem* renderItem;
	float lifetime;
	float age;
	float size;



	Particle(Vector3D Pos, Vector3D Vel, Vector3D Acc, float Damping);
	Particle(Vector3D Pos, Vector3D Vel,Vector3D Acc,float Damping,bool Op); //para euler y verlet
	Particle(const Vector3D& position, const Vector3D& velocity, float mass);


	~Particle();

	void integrate(double t); //mueve en función del tiempo en la dirección de pose
	void integrateSemiImplicitEuler(double t); // (4) semimplicito
	void integrateVerlet(double t); //(4) verlet
	void SetLifeTime(float time);
	bool IsDead() const;

	//p3
	void integrateG(float deltaTime);
	void addForce(const Vector3D& force);

	float getMass() const { return mass; }
	bool isAlive() const { return age < lifetime; }

private:
	Vector3D position;
	Vector3D velocity;
	Vector3D accumulatedForce;  // Fuerza acumulada
	float mass;


};



