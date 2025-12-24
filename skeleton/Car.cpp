#include "Car.h"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace physx;

// Importamos las listas globales para poder añadir el coche al render
extern std::vector<RenderItem*> gRenderItems;
extern std::vector<physx::PxShape*> gShapes;
extern physx::PxMaterial* gMaterial;
extern ForceRegistry* gRegistry;


// =========================================================================
// FILTRO DE RAYCAST (Para que el rayo no choque con el propio chasis)
// =========================================================================
struct IgnoreBodyFilter : public PxQueryFilterCallback
{
	PxRigidActor* actorToIgnore;

	IgnoreBodyFilter(PxRigidActor* actor) : actorToIgnore(actor) {}

	virtual PxQueryHitType::Enum preFilter(
		const PxFilterData& filterData,
		const PxShape* shape,
		const PxRigidActor* actor,
		PxHitFlags& queryFlags)
	{
		if (actor == actorToIgnore) return PxQueryHitType::eNONE;
		return PxQueryHitType::eBLOCK;
	}

	virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit)
	{
		return PxQueryHitType::eBLOCK;
	}
};

// =========================================================================
// IMPLEMENTACIÓN DE LA CLASE CAR
// =========================================================================

Car::Car(PxPhysics* physics, PxScene* scene, const PxTransform& pose, const PxVec3& halfExtents)
	: physics_(physics), scene_(scene), throttle_(0.0f), steer_(0.0f), frameCounter_(0)
{
	moveForce_ = 10000.0f;
	turnTorque_ = 5000.0f;

	carHalfHeight_ = halfExtents.y;

	// Suspensión: Longitud del "rayo" invisible bajo el coche
	suspensionRestLength_ = 2.0f;

	// Fuerza del muelle y amortiguador
	springStrength_ = 60000.0f;
	springDamper_ = 5000.0f;

	actor_ = physics_->createRigidDynamic(pose);

	// Material
	PxMaterial* mat = gMaterial;

	// 1. FORMA VISUAL
	PxShape* shapeVisual = physics_->createShape(PxBoxGeometry(halfExtents), *mat);
	gShapes.push_back(shapeVisual);

	// 2. FORMA FÍSICA (Chasis)
	PxVec3 colliderDim = halfExtents;
	colliderDim.y *= 0.25f; // Muy plano para no chocar fácil con el suelo
	PxShape* shapeCollider = physics_->createShape(PxBoxGeometry(colliderDim), *mat);

	actor_->attachShape(*shapeCollider);
	gShapes.push_back(shapeCollider); // Guardar para limpiar memoria luego

	PxRigidBodyExt::updateMassAndInertia(*actor_, 1500.0f);

	// =================== Cubo arma ===================
	PxVec3 gunHalfExtents(0.2f, 0.4f, 0.6f);

	// IMPORTANTE: exclusive shape (no physics_->createShape)
#include <extensions/PxRigidActorExt.h> // asegúrate de tener este include arriba del cpp

	gunShape_ = PxRigidActorExt::createExclusiveShape(
		*actor_,
		PxBoxGeometry(gunHalfExtents),
		*mat
	);
	gShapes.push_back(gunShape_);

	// Offset local respecto al centro del coche.
	// Tu forward de movimiento es (0,0,-1), o sea que "delante" es -Z:
	PxVec3 const gunOffset(
		0.0f,                        // centrado en X
		halfExtents.y * 0.3f,        // un poco elevado
		-halfExtents.z - gunHalfExtents.z  // delante en -Z
	);
	gunShape_->setLocalPose(PxTransform(gunOffset));

	// =================== TURBO: 2 emisores detrás ===================
	registry_ = gRegistry;

	// offsets locales: atrás es +Z porque tu forward de movimiento es -Z
	// un poco separados en X, a la altura del “escape”
	turboLeftLocalOffset_ = PxVec3(-halfExtents.x * 0.6f, -halfExtents.y * 0.1f, +halfExtents.z + 0.5f);
	turboRightLocalOffset_ = PxVec3(+halfExtents.x * 0.6f, -halfExtents.y * 0.1f, +halfExtents.z + 0.5f);

	// gravedad para el humo/partículas (opcional)
	turboGravity_ = new GravityForceGenerator(Vector3D(0.0, -9.8, 0.0));

	// orientación inicial (local): hacia atrás del coche => +Z local
	Vector3D turboFacingLocal(0, 0, 1);

	// spread pequeñito
	Vector2D spread(12, 12);

	// posición inicial (se corregirá cada frame en update)
	PxTransform poseNow = actor_->getGlobalPose();
	PxVec3 leftWorld = poseNow.transform(turboLeftLocalOffset_);
	PxVec3 rightWorld = poseNow.transform(turboRightLocalOffset_);

	turboLeft_ = new ParticleGenerator(
		/*particlesPerSecond*/ 60,
		spread,
		turboFacingLocal,
		Vector3D(leftWorld.x, leftWorld.y, leftWorld.z)
	);
	turboRight_ = new ParticleGenerator(
		60,
		spread,
		turboFacingLocal,
		Vector3D(rightWorld.x, rightWorld.y, rightWorld.z)
	);

	// config
	turboLeft_->setForceRegistry(registry_);
	turboRight_->setForceRegistry(registry_);

	turboLeft_->setAverageSpeed(45.0f);
	turboRight_->setAverageSpeed(45.0f);

	turboLeft_->setGaussianFactor(2.0);
	turboRight_->setGaussianFactor(2.0);

	turboLeft_->setLifeTime(0.6);
	turboRight_->setLifeTime(0.6);

	turboLeft_->setParticleColor(PxVec4(1.0f, 0.45f, 0.0f, 1.0f));
	turboRight_->setParticleColor(PxVec4(1.0f, 0.45f, 0.0f, 1.0f));

	turboLeft_->setParticleRadius(0.5f);
	turboRight_->setParticleRadius(0.5f);

	// fuerzas del turbo
	turboLeft_->addGlobalForce(turboGravity_);
	turboRight_->addGlobalForce(turboGravity_);

	// arranca activado pero sin emitir
	turboLeft_->setActive(true);
	turboRight_->setActive(true);
	turboLeft_->setEmitting(false);
	turboRight_->setEmitting(false);


	// =================== resto igual ===================

	// Bajar el centro de masas para evitar vuelcos
	actor_->setCMassLocalPose(PxTransform(PxVec3(0.0f, -halfExtents.y, 0.0f)));

	actor_->setLinearDamping(0.1f);
	actor_->setAngularDamping(2.0f); // Alto damping angular para que no gire como peonza

	scene_->addActor(*actor_);

	// Render del chasis
	renderItem_ = new RenderItem(shapeVisual, actor_, PxVec4(0.8f, 0.2f, 0.2f, 1.0f));
	gRenderItems.push_back(renderItem_);

	// Render del cubo arma
	gunRenderItem_ = new RenderItem(gunShape_, actor_, PxVec4(0.0f, 0.0f, 1.0f, 1.0f)); // azul
	gRenderItems.push_back(gunRenderItem_);
}

Car::~Car()
{
	
}

void Car::setThrottle(float v) { throttle_ = v; }
void Car::setSteer(float v) { steer_ = v; }

void Car::setTurbo(bool on)
{
    turboActive_ = on;

    if (turboLeft_)  turboLeft_->setEmitting(on);
    if (turboRight_) turboRight_->setEmitting(on);
}

physx::PxTransform Car::GetGunTransform() const
{
	if (!actor_ || !gunShape_)
		return GetTransform(); // fallback

	PxTransform chassisPose = actor_->getGlobalPose();
	PxTransform localGunPose = gunShape_->getLocalPose();

	return chassisPose * localGunPose;
}

void Car::update(float dt)
{
	if (!actor_ || !scene_) return;

	actor_->wakeUp(); // Asegurar que PhysX no duerma el coche

	PxTransform t = actor_->getGlobalPose();

	// Turbo
	if (turboLeft_ && turboRight_)
	{
		// Posición de cada emisor (local -> mundo)
		PxVec3 leftWorld = t.transform(turboLeftLocalOffset_);
		PxVec3 rightWorld = t.transform(turboRightLocalOffset_);

		turboLeft_->setSpawnPosition(Vector3D(leftWorld.x, leftWorld.y, leftWorld.z));
		turboRight_->setSpawnPosition(Vector3D(rightWorld.x, rightWorld.y, rightWorld.z));

		// Orientación: atrás del coche (tu forward es -Z, así que atrás es +Z local)
		PxVec3 backWorldPx = t.q.rotate(PxVec3(0, 0, 1));
		Vector3D backWorld(backWorldPx.x, backWorldPx.y, backWorldPx.z);

		turboLeft_->setOrientation(backWorld);
		turboRight_->setOrientation(backWorld);

		// Emitir/integrar partículas del turbo (si están apagados, update() retorna)
		turboLeft_->update(dt);
		turboRight_->update(dt);
	}

	frameCounter_++;
	bool doDebug = (frameCounter_ % 60 == 0);

	PxVec3 origin = t.p;

	// Disparamos el rayo hacia ABAJO relativo al coche
	PxVec3 dir = t.q.rotate(PxVec3(0, -1, 0));
	PxVec3 upDir = -dir;

	PxRaycastBuffer hit;
	IgnoreBodyFilter filter(actor_);
	PxQueryFilterData filterData;
	filterData.flags |= PxQueryFlag::ePREFILTER;

	float maxDist = suspensionRestLength_ + 1.0f; // Un poco de margen extra

	bool status = scene_->raycast(origin, dir, maxDist, hit,
		PxHitFlag::eDEFAULT, filterData, &filter);

	bool isGrounded = false;

	if (status)
	{
		float distance = hit.block.distance;

		// Si el suelo está dentro del rango de la suspensión
		if (distance < suspensionRestLength_)
		{
			isGrounded = true;

			float compression = suspensionRestLength_ - distance;

			// Velocidad vertical relativa
			PxVec3 velocity = actor_->getLinearVelocity();
			float springVelocity = velocity.dot(upDir);

			float springForce = (compression * springStrength_);
			float damperForce = (springVelocity * springDamper_);

			float totalForce = springForce - damperForce;

			// === SAFETY KICK CORREGIDO ===
			// Si la distancia es muy pequeña (el chasis va a rozar), empujamos fuerte.
			// carHalfHeight_ es la mitad de la altura, pero el collider es el 25%.
			// Usamos un valor pequeño fijo (0.5m) como límite de pánico.
			if (distance < 0.5f)
			{
				float penetration = 0.5f - distance;
				float penaltyForce = penetration * 100000.0f; // Empuje de emergencia
				totalForce += penaltyForce;

				// if (doDebug) std::cout << " [! SUELO !] ";
			}

			if (totalForce < 0) totalForce = 0; // No succionar

			actor_->addForce(upDir * totalForce * dt, PxForceMode::eIMPULSE);
		}
	}

	// LÓGICA DE MOVIMIENTO (Solo si tocamos suelo)
	if (isGrounded)
	{
		// 1. Fricción Lateral (Para que no deslice como hielo)
		PxVec3 velocity = actor_->getLinearVelocity();
		PxVec3 right = t.q.rotate(PxVec3(1, 0, 0));
		float lateralSpeed = velocity.dot(right);
		// Impulso contrario a la velocidad lateral
		PxVec3 lateralImpulse = -right * lateralSpeed * actor_->getMass() * 0.5f;
		actor_->addForce(lateralImpulse * dt, PxForceMode::eIMPULSE);

		// 2. Aceleración (W / S)
		if (throttle_ != 0.0f) {
			PxVec3 forward = t.q.rotate(PxVec3(0, 0, -1));
			// Nota: Si tu coche va al revés, cambia a PxVec3(0, 0, -1)
			actor_->addForce(forward * throttle_ * moveForce_);

			// if (doDebug) std::cout << "GAS! ";
		}

		// 3. Giro (A / D)
		if (steer_ != 0.0f) {
			PxVec3 up = t.q.rotate(PxVec3(0, 1, 0));
			// Torque para girar sobre el eje Y
			actor_->addTorque(up * (-steer_) * turnTorque_);
		}
	}
}