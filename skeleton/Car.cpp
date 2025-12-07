#include "Car.h"
#include <iostream>
#include <cmath>
#include <algorithm> // Para std::max

extern std::vector<RenderItem*> gRenderItems;
extern std::vector<physx::PxShape*> gShapes;

// =========================================================================
// FILTRO DE RAYCAST
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
// CLASE CAR
// =========================================================================

Car::Car(PxPhysics* physics, PxScene* scene, const PxTransform& pose, const PxVec3& halfExtents)
	: physics_(physics), scene_(scene), throttle_(0.0f), steer_(0.0f), frameCounter_(0)
{
	moveForce_ = 10000.0f;
	turnTorque_ = 5000.0f;

	// Altura visual real (la que queremos proteger que no entre en el suelo)
	carHalfHeight_ = halfExtents.y + 10.0f;

	// === SUSPENSIÓN REFORZADA ===
	// Longitud de reposo amplia para empezar a frenar antes
	suspensionRestLength_ = 1.0f;

	// Fuerza base ALTA
	springStrength_ = 80000.0f;
	springDamper_ = 8000.0f;

	actor_ = physics_->createRigidDynamic(pose);

	// ===========================================================
	// ESTRATEGIA DE SEGURIDAD: COLLIDER ULTRA-FINO
	// ===========================================================

	PxMaterial* mat = physics_->createMaterial(0.0f, 0.0f, 0.0f);

	// 1. FORMA VISUAL (Tamaño completo para el render)
	PxShape* shapeVisual = physics_->createShape(PxBoxGeometry(halfExtents), *mat);
	gShapes.push_back(shapeVisual);

	// 2. FORMA FÍSICA (Collider reducido al 25%)
	// Al ser tan fino, el centro físico está muy lejos del suelo incluso
	// cuando la rueda visual está tocando.
	PxVec3 colliderDim = halfExtents;
	colliderDim.y *= 0.25f; // <--- MUY FINO (Como un chasis de skate)

	PxShape* shapeCollider = physics_->createShape(PxBoxGeometry(colliderDim), *mat);
	actor_->attachShape(*shapeCollider);
	gShapes.push_back(shapeCollider);

	// ===========================================================

	PxRigidBodyExt::updateMassAndInertia(*actor_, 1500.0f);
	// Centro de masas en la parte baja visual
	actor_->setCMassLocalPose(PxTransform(PxVec3(0.0f, -halfExtents.y, 0.0f)));

	actor_->setLinearDamping(0.1f);
	actor_->setAngularDamping(0.8f);

	scene_->addActor(*actor_);

	renderItem_ = new RenderItem(shapeVisual, actor_, PxVec4(0.8f, 0.2f, 0.2f, 1.0f));
	gRenderItems.push_back(renderItem_);
}

Car::~Car()
{
}

void Car::setThrottle(float v) { throttle_ = v; }
void Car::setSteer(float v) { steer_ = v; }

void Car::update(float dt)
{
	if (!actor_ || !scene_) return;

	actor_->wakeUp();

	frameCounter_++;
	bool doDebug = (frameCounter_ % 60 == 0);

	PxTransform t = actor_->getGlobalPose();
	PxVec3 origin = t.p;
	PxVec3 dir = t.q.rotate(PxVec3(0, -1, 0));
	PxVec3 upDir = -dir;

	PxRaycastBuffer hit;
	IgnoreBodyFilter filter(actor_);
	PxQueryFilterData filterData;
	filterData.flags |= PxQueryFlag::ePREFILTER;

	// Raycast largo para anticipar suelo
	float maxDist = suspensionRestLength_ + 2.0f;

	bool status = scene_->raycast(origin, dir, maxDist, hit,
		PxHitFlag::eDEFAULT, filterData, &filter);

	bool isGrounded = false;

	if (status)
	{
		float distance = hit.block.distance;

		if (distance < suspensionRestLength_)
		{
			isGrounded = true;

			float compression = suspensionRestLength_ - distance;

			PxVec3 velocity = actor_->getLinearVelocity();
			float springVelocity = velocity.dot(upDir);

			float springForce = (compression * springStrength_);
			float damperForce = (springVelocity * springDamper_);

			float totalForce = springForce - damperForce;

			// === SEGURIDAD ANTI-CLIPPING (BUMP STOP) ===
			// carHalfHeight_ es la distancia desde el centro hasta la panza visual del coche.
			// Si la distancia es menor que eso, ESTAMOS ENTRANDO EN EL SUELO.

			// Zona de peligro: Empezamos a aplicar fuerza extra un poco antes de tocar (al 120% de la altura)
			float limitDist = carHalfHeight_ * 1.2f;

			if (distance < limitDist)
			{
				// Calculamos cuánto nos estamos pasando de la raya
				float penetration = limitDist - distance;

				// Fuerza exponencial: Cuanto más entra, más bestia es la respuesta.
				// Multiplicamos por 200,000 extra por cada metro de penetración
				float penaltyForce = penetration * 500000.0f;

				totalForce += penaltyForce;

				if (doDebug) std::cout << " [!!! SAFETY KICK: " << penaltyForce << " !!!]";
			}
			// ==========================================

			if (doDebug) {
				std::cout << "D: " << distance << " | F: " << totalForce << std::endl;
			}

			// Aseguramos que la fuerza nunca sea negativa (no succionar hacia el suelo)
			if (totalForce < 0) totalForce = 0;

			actor_->addForce(upDir * totalForce * dt, PxForceMode::eIMPULSE);
		}
	}

	if (isGrounded)
	{
		// Fricción lateral
		PxVec3 velocity = actor_->getLinearVelocity();
		PxVec3 right = t.q.rotate(PxVec3(1, 0, 0));
		float lateralSpeed = velocity.dot(right);
		PxVec3 lateralImpulse = -right * lateralSpeed * actor_->getMass() * 0.8f;
		actor_->addForce(lateralImpulse * dt, PxForceMode::eIMPULSE);

		// Controles
		if (throttle_ != 0.0f) {
			PxVec3 forward = t.q.rotate(PxVec3(0, 0, 1));
			actor_->addForce(forward * throttle_ * moveForce_);
		}

		if (steer_ != 0.0f) {
			PxVec3 up = t.q.rotate(PxVec3(0, 1, 0));
			actor_->addTorque(up * (-steer_) * turnTorque_);
		}
	}
}