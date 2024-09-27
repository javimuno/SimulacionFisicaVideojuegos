#include "Particle.h"


Particle::Particle(Vector3D Pos, Vector3D Vel)
{
	vel = Vel;
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
	pose = physx::PxTransform(pose.p.x + vel.x * t, pose.p.y + vel.y * t, pose.p.z + vel.z * t);
}