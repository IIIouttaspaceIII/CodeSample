#pragma once
#include "FlecsStructs.h"

struct VelocityDeltaPair {
	float aDelta;
	float bDelta;
};

struct FrictionResult {
	float appliedForce;
	float deltaVelocity;
};

struct VelocityDeltaTrio {
	float inputDelta;
	float aDelta;
	float bDelta;
};

VelocityDeltaTrio SolveDifferentialConnection(const GearInverseMomentum& inputB, const GearVelocity& inputV, const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV);

VelocityDeltaPair SolveGearConnection(const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV, const float& gearing);

VelocityDeltaPair SolveClutchConnection(const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV, const float& gearing, const float& maxTorque, const float& deltaTime);

FrictionResult SolveFrictionConnection(const GearInverseMomentum& aB, GearVelocity& aV, const float& maxTorque, const float& goalVelocity, const float& deltaTime);