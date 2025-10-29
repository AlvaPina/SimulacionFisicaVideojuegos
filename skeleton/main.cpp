#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>

#include "Axis.h"
#include "ParticleGenerator.h"
#include "Particle.h"
#include "Vector2D.h"
#include "Vector3D.h"
std::string display_text = "Hola Mundo";


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

std::vector<Particle*> gParticles;
std::vector<ParticleGenerator*> gParticleGenerators;
std::vector<physx::PxShape*> gShapes;
std::vector<RenderItem*> gRenderItems;

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

	// Axis
	Axis axis(10.f);

	// Particula
	Vector2D spreadAngle = Vector2D(30, 30);
	Vector2D orientation = Vector2D::UP;
	ParticleGenerator* generador = new ParticleGenerator(10, spreadAngle, orientation);
}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	gScene->simulate(t);
	gScene->fetchResults(true);

	// Actualizamos particulas
	for (Particle* particle : gParticles) {
		if (particle) {
			//std::cout << "integrate";
			particle->integrate(t);
		}
	}
	// Actualizamos generadores particulas
	for (ParticleGenerator* generator : gParticleGenerators) {
		if (generator) {
			//std::cout << "integrate";
			generator->update(t);
		}
	}
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

	// Release Particles
	for (Particle* particle : gParticles) {
		if (particle) {
			delete particle;
		}
	}
	gParticles.clear();

	// Release Particles Generators
	for (ParticleGenerator* generator : gParticleGenerators) {
		if (generator) {
			delete generator;
		}
	}
	gParticleGenerators.clear();

	// Release RenderItems (Deregister + delete)
	for (RenderItem* item : gRenderItems) {
		if (item) {
			DeregisterRenderItem(item);
			delete item;
		}
	}
	gRenderItems.clear();

	// Release Shapes de PhysX (release())
	for (physx::PxShape* shape : gShapes) {
		if (shape) shape->release();
	}
	gShapes.clear();
}

// Function called when a key is pressed
void keyPress(unsigned char key, const PxTransform& camera)
{
	PX_UNUSED(camera);

	switch(toupper(key))
	{
	case 'B': {
		std::cout << "camera.q = ("
			<< camera.q.x << ", "
			<< camera.q.y << ", "
			<< camera.q.z << ")"
			<< std::endl;
		Vector3D forward = camera.q.rotate(PxVec3(0, 0, -1));
		Vector3D iniPos = Vector3D(camera.p.x, camera.p.y, camera.p.z);
		float iniVel = 10.0f;
		Particle* particle = new Particle(iniPos, forward.scalarMul(iniVel), 1.0f);
		particle->setGravity(false);
		gParticles.push_back(particle);
		break;
	}
	case ' ': {
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