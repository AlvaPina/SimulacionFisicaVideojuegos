#pragma once
#include <vector>
#include <PxPhysicsAPI.h>
#include "RenderUtils.hpp"

// forward
class ForceRegistry;
class ForceGenerator;
class GravityForceGenerator;
class VortexForceGenerator;

class MapGenerator
{
public:
    struct Settings
    {
        physx::PxVec3 halfExtents = physx::PxVec3(10.f, 1.f, 10.f);
        float yawStepDeg = 30.0f;
        float maxYawDeg = 90.0f;
        float slopeAngleDeg = 12.0f;
        unsigned seed = 1234;

        int wStraight = 55;
        int wTurnLeft = 15;
        int wTurnRight = 15;
        int wSlopeUp = 8;
        int wSlopeDown = 7;
        int wStraightAfterSlope = 75;

        int   tornadoEvery = 10;         // cada N suelos (0 = desactivado)
        float tornadoSideOffset = 22.0f; // distancia lateral desde el centro del tile
        float tornadoForwardJitter = 0.0f; // si quieres desplazar un poco en forward
        bool  tornadoRandomSide = true; // si no, alterna izq/der
    };

    enum class TileType
    {
        STRAIGHT,
        TURN_LEFT,
        TURN_RIGHT,
        SLOPE_UP,
        SLOPE_DOWN
    };

public:
    MapGenerator(physx::PxPhysics* physics, physx::PxScene* scene, physx::PxMaterial* material, const Settings& settings);
    ~MapGenerator();

    void generate(int numTiles, const physx::PxTransform& startPose);
    void clear();

private:
    physx::PxQuat yawDeg(float deg) const;
    physx::PxQuat pitchDeg(float deg) const;

    TileType pickNextType(int i, TileType prev, float currentYawDeg);

    void createTileActor(const physx::PxTransform& tilePose);

    void maybeSpawnTornado(int tileIndex, const physx::PxTransform& tilePose, const physx::PxQuat& yawQ);
    void spawnTornadoAt(const physx::PxVec3& center, float radius);

private:
    physx::PxPhysics* physics_ = nullptr;
    physx::PxScene* scene_ = nullptr;
    physx::PxMaterial* material_ = nullptr;
    Settings settings_;

    std::vector<physx::PxRigidStatic*> tiles_;

    physx::PxTransform cursor_{ physx::PxIdentity };
    float yawAccumDeg_ = 0.0f;
    float heightAccum_ = 0.0f;

    TileType prevType_ = TileType::STRAIGHT;

    // Tornados creados (para poder limpiarlos)
    std::vector<class ParticleGenerator*> tornadoEmitters_;
    std::vector<ForceGenerator*> tornadoForces_;
};
