#pragma once

#include "Vector3D.h"
#include "RenderUtils.hpp"
#include "core.hpp"

class Particle
{

public:

	Particle(Vector3D Pos, Vector3D Vel);
	~Particle();

	void integrate(double t); //mueve en función del tiempo en la dirección de pose

private:

	Vector3D vel;
	physx::PxTransform pose; //a render item le pasamos la direccion de esta pose
								//para que se actualice automaticamente
	RenderItem* renderItem;

};

