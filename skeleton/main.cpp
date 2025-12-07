#include <ctype.h>

#include <PxPhysicsAPI.h>

#include <vector>

#include "core.hpp"
#include "RenderUtils.hpp"
#include "callbacks.hpp"

#include <iostream>

#include "Axis.h"
#include "ParticleGenerator.h"
#include "GravityForceGenerator.h"
#include "WindForceGenerator.h"
#include "VortexForceGenerator.h"
#include "ExplosionForceGenerator.h"
#include "SpringForceGenerator.h"
#include "RigidBody.h"
#include "RigidBodyPhysX.h"
#include "RigidBodyPhysXStatic.h"
#include "Particle.h"
#include "Car.h"
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

ForceRegistry* gRegistry = nullptr;

ExplosionForceGenerator* gExplosionFG = nullptr;

RigidBody* gSpringBody = nullptr;   // el cuerpo que se mueve
RigidBody* gSpringAnchor = nullptr;   // el punto de anclaje
SpringForceGenerator* gSpringFG = nullptr;

RigidBody* gCubeRB = nullptr;
SpringForceGenerator* gCubeSpringFG = nullptr;
RigidBody* gCubeAnchorRB = nullptr;

Car* gCar = nullptr;


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

	// Crear force registry
	gRegistry = new ForceRegistry();

	// =================== Generador base ===================
	Vector2D spreadAngle = Vector2D(10, 10);
	Vector3D orientation = Vector3D::FORWARD;
	Vector3D spawnPos = Vector3D(20, 0, 0);

	ParticleGenerator* generadorBase = new ParticleGenerator(5, spreadAngle, orientation, spawnPos);
	generadorBase->setAverageSpeed(10.0);
	generadorBase->setGaussianFactor(1.0);
	generadorBase->setLifeTime(2.0);
	gParticleGenerators.push_back(generadorBase);

	// Conectar registry al emisor base
	generadorBase->setForceRegistry(gRegistry);

	// Gravedad (global)
	auto* gravityFG = new GravityForceGenerator(Vector3D(0.0, -9.8, 0.0));
	generadorBase->addGlobalForce(gravityFG);

	// =================== Generador de partículas para explosión ===================
	spreadAngle = Vector2D(360, 360);
	orientation = Vector3D::UP;
	spawnPos = Vector3D(0, 0, 0);

	ParticleGenerator* generadorExplosion = new ParticleGenerator(20, spreadAngle, orientation, spawnPos);
	generadorExplosion->setAverageSpeed(10.0);
	generadorExplosion->setGaussianFactor(1.0);
	generadorExplosion->setLifeTime(2.0);
	generadorExplosion->setForceRegistry(gRegistry);
	generadorExplosion->addGlobalForce(gravityFG);
	gParticleGenerators.push_back(generadorExplosion);

	// =================== Generador con viento (nuevo emisor) ===================
	spreadAngle = Vector2D(20, 20);
	orientation = Vector3D::FORWARD;
	spawnPos = Vector3D(40, 0, 0);

	ParticleGenerator* generadorViento = new ParticleGenerator(20, spreadAngle, orientation, spawnPos);
	generadorViento->setAverageSpeed(12.0);
	generadorViento->setGaussianFactor(1.0);
	generadorViento->setLifeTime(2.0);
	generadorViento->setForceRegistry(gRegistry);
	generadorViento->addGlobalForce(gravityFG);
	gParticleGenerators.push_back(generadorViento);

	// ---- VIENTO (AABB) ----
	auto* windFG = new WindForceGenerator(Vector3D(8, 0, 0), /*k1*/0.8, /*k2*/0.0);
	windFG->setVolume(Vector3D(-10, -10, -10), Vector3D(10, 10, 10));

	// Afecta al emisor base y al de viento
	generadorBase->addGlobalForce(windFG);
	generadorViento->addGlobalForce(windFG);

	// =================== Generador con torbellino (nuevo emisor) ===================
	spreadAngle = Vector2D(25, 25);
	orientation = Vector3D::UP;
	spawnPos = Vector3D(0, 0, 0);

	ParticleGenerator* generadorVortex = new ParticleGenerator(20, spreadAngle, orientation, spawnPos);
	generadorVortex->setAverageSpeed(8.0);
	generadorVortex->setGaussianFactor(1.0);
	generadorVortex->setLifeTime(20.0);
	generadorVortex->setForceRegistry(gRegistry);
	generadorVortex->addGlobalForce(gravityFG);
	gParticleGenerators.push_back(generadorVortex);

	// ---- TORBELLINO (eje Y, radio 8) ----
	auto* vortexFG = new VortexForceGenerator(/*K*/ 4.0,
		/*center*/ Vector3D(0, 0, 0),
		/*radius*/ 25.0,
		/*axis*/   Vector3D(0, 1, 0));
	vortexFG->setK1(0.8);
	vortexFG->setK2(0.0);

	// Afecta al emisor base y al de torbellino
	generadorBase->addGlobalForce(vortexFG);
	generadorVortex->addGlobalForce(vortexFG);

	// =================== Explosión (OFF al inicio; se activa con tecla) ===================
	gExplosionFG = new ExplosionForceGenerator(/*center*/ Vector3D(0, 0, 0),
		/*K*/      2000.0,
		/*R0*/     6.0,
		/*tau*/    1.0,
		/*ve*/     0.0);
	gExplosionFG->setActive(false);

	// Afecta al emisor base y al de explosión
	generadorBase->addGlobalForce(gExplosionFG);
	generadorExplosion->addGlobalForce(gExplosionFG);

	// =================== MUELLE ===================

	// 1) Partícula ancla (estática "de facto" porque no le aplicamos fuerzas)
	gSpringAnchor = new Particle(
		Vector3D(-50.0, 5.0, 0.0),   // posición inicial
		Vector3D(0.0, 0.0, 0.0),   // velocidad inicial
		1.0                        // masa (da igual, no recibe fuerzas)
	);
	gParticles.push_back(static_cast<Particle*>(gSpringAnchor)); // para que se dibuje e integre (no se moverá)

	// 2) Partícula colgante
	gSpringBody = new Particle(
		Vector3D(-50.0, 1.0, 0.0),   // por debajo del ancla
		Vector3D(0.0, 100.0, 0.0),   // velocidad inicial
		1.0                        // masa
	);
	gParticles.push_back(static_cast<Particle*>(gSpringBody));

	// 3) Muelle entre ancla y cuerpo
	double k = 80.0;          // constante elástica
	double resting_length = 4.0; // longitud de reposo (distancia entre 5 y 1 en Y = 4)

	gSpringFG = new SpringForceGenerator(k, resting_length, gSpringAnchor);

	// =================== Cubo rígido colgante ===================
	PxTransform cubeTransform(PxVec3(-40.0f, 3.0f, 0.0f));
	PxRigidDynamic* cubeActor = gPhysics->createRigidDynamic(cubeTransform);

	PxShape* cubeShape = CreateShape(PxBoxGeometry(2.0f, 2.0f, 2.0f));
	cubeActor->attachShape(*cubeShape);

	PxRigidBodyExt::updateMassAndInertia(*cubeActor, 1.0f);
	gScene->addActor(*cubeActor);

	// Render del cubo
	RenderItem* cubeItem = new RenderItem(cubeShape, cubeActor, PxVec4(0, 1, 0, 1));
	gRenderItems.push_back(cubeItem);

	// Adaptador dinámico
	gCubeRB = new RigidBodyPhysX(cubeActor);

	// ---- Ancla fija con PxRigidStatic ----
	PxRigidStatic* anchorActor = gPhysics->createRigidStatic(PxTransform(PxVec3(-40, 15, 0)));
	PxShape* anchorShape = CreateShape(PxSphereGeometry(0.2f));
	anchorActor->attachShape(*anchorShape);
	gScene->addActor(*anchorActor);

	// Render de la bolita del ancla
	RenderItem* anchorItem = new RenderItem(anchorShape, anchorActor, PxVec4(1, 0, 0, 1));
	gRenderItems.push_back(anchorItem);

	// Adaptador estático
	gCubeAnchorRB = new RigidBodyPhysXStatic(anchorActor);

	// ---- Muelle entre ancla estática y cubo dinámico ----
	double kCube = 80.0;
	double restCube = 10.0;

	gCubeSpringFG = new SpringForceGenerator(kCube, restCube, gCubeAnchorRB);

	// =================== Suelo estático ===================
	PxRigidStatic* groundActor =
		gPhysics->createRigidStatic(PxTransform(PxVec3(0.0f, -2.0f, 0.0f)));

	PxShape* groundShape = CreateShape(PxBoxGeometry(50.0f, 1.0f, 50.0f));
	groundActor->attachShape(*groundShape);
	gScene->addActor(*groundActor);

	// Render del suelo
	RenderItem* groundItem = new RenderItem(groundShape, groundActor,
		PxVec4(0.3f, 0.3f, 0.3f, 1.0f));
	gRenderItems.push_back(groundItem);

	// =================== Coche ===================
	PxTransform carPose(PxVec3(0.0f, 2.0f, -20.0f));
	PxVec3 carHalfExtents(2.0f, 0.5f, 4.0f);   // ancho, alto, largo

	gCar = new Car(gPhysics, gScene, carPose, carHalfExtents);
}


// Function to configure what happens in each step of physics
// interactive: true if the game is rendering, false if it offline
// t: time passed since last call in milliseconds
void stepPhysics(bool interactive, double t)
{
	PX_UNUSED(interactive);

	// 1) Aplicar fuerzas de muelles (antes de simular PhysX)
	if (gSpringFG && gSpringBody)
		gSpringFG->apply(*gSpringBody, t);      // muelle de partículas

	if (gCubeSpringFG && gCubeRB)
		gCubeSpringFG->apply(*gCubeRB, t);      // muelle del cubo PhysX

	// 2) Simulación PhysX
	gScene->simulate(t);
	gScene->fetchResults(true);

	// 3) Partículas "caseras"
	for (Particle* particle : gParticles)
		if (particle) particle->integrate(t);

	for (ParticleGenerator* generator : gParticleGenerators)
		if (generator) generator->update(t);

	if (gCar) gCar->update(static_cast<float>(t));

	if (gCar) gCar->update(static_cast<float>(t));

	// CÁMARA SIGUE AL COCHE
	if (interactive && gCar)
	{
		// 1. Obtener la posición del coche
		PxTransform carPose = gCar->GetTransform();

		// 2. Calcular posición deseada de la cámara
		// Offset: 15 metros detrás, 5 arriba.
		// Si la cámara mira al revés, cambia 15.0f por -15.0f
		PxVec3 offset(0.0f, 6.0f, 15.0f);

		// Convertir offset local a mundo (para que rote con el coche)
		PxVec3 camPos = carPose.transform(offset);

		// 3. Calcular vector dirección (Cámara mira al coche)
		// Destino (Coche) - Origen (Cámara)
		PxVec3 camDir = (carPose.p - camPos).getNormalized();

		// 4. Aplicar cambios
		Camera* cam = GetCamera();
		if (cam)
		{
			cam->setEye(camPos);
			cam->setDir(camDir);
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

	if (gSpringFG) {
		delete gSpringFG;
		gSpringFG = nullptr;
	}
	gSpringBody = nullptr;   // ya se borró en el bucle de gParticles
	gSpringAnchor = nullptr; // idem

	if (gCar) {
		delete gCar;
		gCar = nullptr;
	}
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
		gParticles.push_back(particle);
		break;
	}
	case 'E': {
		if (gExplosionFG) {
			gExplosionFG->resetTime();
			gExplosionFG->setActive(true);
			std::cout << "Explosión activada\n";
		}
		break;
	}
	case 'W':    // acelerar
		if (gCar) gCar->setThrottle(3.0f);
		break;
	case 'S':    // marcha atrás / freno
		if (gCar) gCar->setThrottle(-3.0f);
		break;
	case 'A':    // girar izquierda
		if (gCar) gCar->setSteer(-1000.0f);
		break;
	case 'D':    // girar derecha
		if (gCar) gCar->setSteer(1000.0f);
		break;
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