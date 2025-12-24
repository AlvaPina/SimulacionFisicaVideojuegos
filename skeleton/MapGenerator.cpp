#include "MapGenerator.h"
#include <random>
#include <cmath>

#include "ParticleGenerator.h"
#include "ForceRegistry.h"
#include "GravityForceGenerator.h"
#include "VortexForceGenerator.h"

// Globales (como lo llevas tú)
extern std::vector<RenderItem*> gRenderItems;
extern std::vector<physx::PxShape*> gShapes;
extern std::vector<ParticleGenerator*> gParticleGenerators;
extern ForceRegistry* gRegistry;

using namespace physx;

MapGenerator::MapGenerator(PxPhysics* physics, PxScene* scene, PxMaterial* material, const Settings& settings)
    : physics_(physics), scene_(scene), material_(material), settings_(settings)
{
}

MapGenerator::~MapGenerator()
{
    clear();
}

PxQuat MapGenerator::yawDeg(float deg) const
{
    const float rad = deg * PxPi / 180.0f;
    return PxQuat(rad, PxVec3(0, 1, 0));
}

PxQuat MapGenerator::pitchDeg(float deg) const
{
    const float rad = deg * PxPi / 180.0f;
    return PxQuat(rad, PxVec3(1, 0, 0));
}

void MapGenerator::clear()
{
    tiles_.clear();

    // OJO: tus shapes/renderitems se liberan globalmente en cleanupPhysics.
    // Aquí sí borramos “lo nuestro” (emitters + forces) para que no se acumulen si regeneras mapa.
    for (auto* pg : tornadoEmitters_) {
        // también están en gParticleGenerators: quítalos de ahí para que no los actualicen más
        auto it = std::find(gParticleGenerators.begin(), gParticleGenerators.end(), pg);
        if (it != gParticleGenerators.end()) gParticleGenerators.erase(it);

        delete pg;
    }
    tornadoEmitters_.clear();

    for (auto* fg : tornadoForces_) delete fg;
    tornadoForces_.clear();
}

MapGenerator::TileType MapGenerator::pickNextType(int i, TileType prev, float currentYawDeg)
{
    static thread_local std::mt19937 rng;
    rng.seed(settings_.seed + 2654435761u * (unsigned)i);

    const bool canTurnLeft = (currentYawDeg + settings_.yawStepDeg <= settings_.maxYawDeg + 0.001f);
    const bool canTurnRight = (currentYawDeg - settings_.yawStepDeg >= -settings_.maxYawDeg - 0.001f);

    int wS = settings_.wStraight;
    int wL = canTurnLeft ? settings_.wTurnLeft : 0;
    int wR = canTurnRight ? settings_.wTurnRight : 0;
    int wU = settings_.wSlopeUp;
    int wD = settings_.wSlopeDown;

    if (prev == TileType::SLOPE_UP || prev == TileType::SLOPE_DOWN) {
        wS = settings_.wStraightAfterSlope;
        wL = canTurnLeft ? (wL / 2) : 0;
        wR = canTurnRight ? (wR / 2) : 0;
        wU = (wU / 2);
        wD = (wD / 2);
    }

    const int total = wS + wL + wR + wU + wD;
    if (total <= 0) return TileType::STRAIGHT;

    std::uniform_int_distribution<int> dist(1, total);
    int r = dist(rng);

    if ((r -= wS) <= 0) return TileType::STRAIGHT;
    if ((r -= wL) <= 0) return TileType::TURN_LEFT;
    if ((r -= wR) <= 0) return TileType::TURN_RIGHT;
    if ((r -= wU) <= 0) return TileType::SLOPE_UP;
    return TileType::SLOPE_DOWN;
}

void MapGenerator::createTileActor(const PxTransform& tilePose)
{
    PxRigidStatic* a = physics_->createRigidStatic(tilePose);

    PxShape* shape = physics_->createShape(PxBoxGeometry(settings_.halfExtents), *material_);
    a->attachShape(*shape);

    gShapes.push_back(shape);

    scene_->addActor(*a);
    tiles_.push_back(a);

    RenderItem* it = new RenderItem(shape, a, PxVec4(0.45f, 0.45f, 0.45f, 1.0f));
    gRenderItems.push_back(it);
}

// ===== Tornados =====

void MapGenerator::spawnTornadoAt(const PxVec3& center, float radius)
{
    // Emisor visual
    Vector2D spread(25, 25);
    Vector3D orientation = Vector3D::UP;
    Vector3D spawnPos(center.x, center.y, center.z);

    ParticleGenerator* gen = new ParticleGenerator(/*pps*/ 20, spread, orientation, spawnPos);
    gen->setForceRegistry(gRegistry);
    gen->setAverageSpeed(8.0);
    gen->setGaussianFactor(1.0);
    gen->setLifeTime(6.0);
    gen->setEmitting(true);
    gen->setActive(true);

    // Fuerzas del tornado
    auto* gravityFG = new GravityForceGenerator(Vector3D(0.0, 9.8, 0.0));
    auto* vortexFG = new VortexForceGenerator(
        /*K*/      4.0,
        /*center*/ Vector3D(center.x, center.y, center.z),
        /*radius*/ radius,
        /*axis*/   Vector3D(0, 1, 0)
    );
    vortexFG->setK1(0.8);
    vortexFG->setK2(0.0);

    gen->addGlobalForce(gravityFG);
    gen->addGlobalForce(vortexFG);

    // Guardamos para limpieza
    tornadoEmitters_.push_back(gen);
    tornadoForces_.push_back(gravityFG);
    tornadoForces_.push_back(vortexFG);

    // En tu bucle global de update()
    gParticleGenerators.push_back(gen);
}

void MapGenerator::maybeSpawnTornado(int tileIndex, const PxTransform& tilePose, const PxQuat& yawQ)
{
    if (settings_.tornadoEvery <= 0) return;
    if (tileIndex == 0) return; // si no quieres en el primero
    if ((tileIndex % settings_.tornadoEvery) != 0) return;

    // Derecha del tile (en mundo) usando yaw (no pitch)
    PxVec3 right = yawQ.rotate(PxVec3(1, 0, 0));
    right.normalize();

    // determinista para elegir lado si random
    bool placeRight = true;
    if (settings_.tornadoRandomSide) {
        std::mt19937 rng(settings_.seed + 99991u * (unsigned)tileIndex);
        std::uniform_int_distribution<int> d(0, 1);
        placeRight = (d(rng) == 1);
    }
    else {
        // alterna
        placeRight = ((tileIndex / settings_.tornadoEvery) % 2) == 0;
    }

    float sideSign = placeRight ? +1.0f : -1.0f;

    // Un poco a un lado del tile, a la misma altura del tile
    PxVec3 center = tilePose.p + right * (settings_.tornadoSideOffset * sideSign);

    // Si quieres que esté “un pelín adelantado/atrasado”
    if (std::abs(settings_.tornadoForwardJitter) > 0.0001f) {
        PxVec3 forward = yawQ.rotate(PxVec3(0, 0, -1));
        forward.normalize();
        center += forward * settings_.tornadoForwardJitter;
    }

    // Radio del vortex (puedes relacionarlo con tamaño del tile)
    float radius = settings_.halfExtents.x * 2.0f; // ejemplo

    spawnTornadoAt(center, radius);
}

void MapGenerator::generate(int numTiles, const PxTransform& startPose)
{
    clear();

    cursor_ = startPose;
    yawAccumDeg_ = 0.0f;
    heightAccum_ = 0.0f;
    prevType_ = TileType::STRAIGHT;

    const float tileLen = settings_.halfExtents.z * 2.0f;
    const float slopeRad = settings_.slopeAngleDeg * PxPi / 180.0f;

    // subida por tile (signo lo ponemos según tipo)
    const float risePerTile = std::sin(slopeRad) * tileLen;

    for (int i = 0; i < numTiles; ++i)
    {
        TileType type = pickNextType(i, prevType_, yawAccumDeg_);

        float tilePitchDeg = 0.0f;

        if (type == TileType::TURN_LEFT) {
            yawAccumDeg_ += settings_.yawStepDeg;
            if (yawAccumDeg_ > settings_.maxYawDeg) yawAccumDeg_ = settings_.maxYawDeg;
        }
        else if (type == TileType::TURN_RIGHT) {
            yawAccumDeg_ -= settings_.yawStepDeg;
            if (yawAccumDeg_ < -settings_.maxYawDeg) yawAccumDeg_ = -settings_.maxYawDeg;
        }
        else if (type == TileType::SLOPE_UP) {
            tilePitchDeg = +settings_.slopeAngleDeg; // (según tu convención actual)
        }
        else if (type == TileType::SLOPE_DOWN) {
            tilePitchDeg = -settings_.slopeAngleDeg;
        }

        PxQuat yawQ = startPose.q * yawDeg(yawAccumDeg_);

        PxVec3 forwardYaw = yawQ.rotate(PxVec3(0, 0, -1));
        forwardYaw.normalize();

        PxTransform tilePose = cursor_;
        tilePose.p.y = startPose.p.y + heightAccum_;

        // CENTRO ajustado para que el borde de entrada conecte sin escalón
        if (type == TileType::SLOPE_UP) {
            tilePose.p.y += (risePerTile * 0.5f);
        }
        else if (type == TileType::SLOPE_DOWN) {
            tilePose.p.y -= (risePerTile * 0.5f);
        }

        tilePose.q = yawQ * pitchDeg(tilePitchDeg);

        createTileActor(tilePose);

        maybeSpawnTornado(i, tilePose, yawQ);

        // siguiente centro
        PxVec3 nextCenter = tilePose.p + forwardYaw * tileLen;

        if (type == TileType::SLOPE_UP) {
            heightAccum_ += risePerTile;
        }
        else if (type == TileType::SLOPE_DOWN) {
            heightAccum_ -= risePerTile;
        }

        nextCenter.y = startPose.p.y + heightAccum_;
        cursor_ = PxTransform(nextCenter, yawQ);

        prevType_ = type;
    }
}
