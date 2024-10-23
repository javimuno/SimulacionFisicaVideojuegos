#include "Particle.h"


Particle::Particle(Vector3D Pos, Vector3D Vel,Vector3D Acc, float Damping)
{
	vel = Vel;//(2)
	acc = Acc; //(3)
	damping = Damping;
	pose = physx::PxTransform(Pos.x, Pos.y, Pos.z);
	renderItem = new RenderItem(CreateShape(physx::PxSphereGeometry(1.0)), &pose, { 0.1,0.9,0.9,1 });
	RegisterRenderItem(renderItem);
}

//Contructor para Euler y Verlet

Particle::Particle(Vector3D Pos, Vector3D Vel, Vector3D Acc, float Damping, bool Op)
	{
	vel = Vel;//(2)
	acc = Acc; //(3)
	damping = Damping;
	prevPos = Pos;
	op = Op;
	pose = physx::PxTransform(Pos.x, Pos.y, Pos.z);
	renderItem = new RenderItem(CreateShape(physx::PxSphereGeometry(1.0)), &pose, { 0.5, 0.4, 0, 1 });
	RegisterRenderItem(renderItem);
}

Particle::~Particle()
{
	DeregisterRenderItem(renderItem);
}

void Particle::integrate(double t)
{

	//(3)
	//velocidad con aceleracion
	vel.x += acc.x * t;
	vel.y += acc.y * t;
	vel.z += acc.z * t;
	//damping a la velocidad
	vel.x *= pow(damping, t);
	vel.y *= pow(damping, t);
	vel.z *= pow(damping, t);
	//posicion con velocidad actualizada
	pose = physx::PxTransform(pose.p.x + vel.x * t, pose.p.y + vel.y * t, pose.p.z + vel.z * t);

	age += t;

}

//Euler Semi-implicito

void Particle::integrateSemiImplicitEuler(double t) {
	// Actualizar la velocidad con aceleración y damping
	vel = vel + (acc * t);
	vel = vel * pow(damping, t);  // damping

	// Actualizar la posición con la nueva velocidad
	pose.p.x += vel.x * t;
	pose.p.y += vel.y * t;
	pose.p.z += vel.z * t;

	// Actualizar la representación gráfica
	pose = physx::PxTransform(pose.p.x, pose.p.y, pose.p.z);
}



//Verlet

void Particle::integrateVerlet(double t) {
	// Guardar la posición actual
	Vector3D currentPos = Vector3D(pose.p.x, pose.p.y, pose.p.z);

	// Calcular la nueva posición
	pose.p.x = currentPos.x + (currentPos.x - prevPos.x) + acc.x * t * t;
	pose.p.y = currentPos.y + (currentPos.y - prevPos.y) + acc.y * t * t;
	pose.p.z = currentPos.z + (currentPos.z - prevPos.z) + acc.z * t * t;

	// Aplicar damping a la velocidad implícita en el movimiento
	vel = vel * pow(damping, t);

	// Actualizar la posición anterior para el próximo paso
	prevPos = currentPos;

	// Actualizar la representación gráfica
	pose = physx::PxTransform(pose.p.x, pose.p.y, pose.p.z);
}

void Particle::SetLifeTime(float time)
{
	lifetime = time;
}


// Método para verificar si la partícula ha "muerto" (es decir, si su edad supera su vida útil)
bool Particle::IsDead() const {
	return age >= lifetime;
}

