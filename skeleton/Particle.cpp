#include "Particle.h"


Particle::Particle(Vector3D Pos, Vector3D Vel,Vector3D Acc, float Damping)
{
	vel = Vel;//(2)
	acc = Acc; //(3)
	damping = Damping;
	pose = physx::PxTransform(Pos.x, Pos.y, Pos.z);
	renderItem = new RenderItem(CreateShape(physx::PxSphereGeometry(1.0)), &pose, { 0.5,0.4,0,1 });
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

}