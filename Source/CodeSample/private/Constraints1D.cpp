#include "constraints1D.h"
#include "FlecsSubsystem.h"

VelocityDeltaTrio SolveDifferentialConnection(const GearInverseMomentum& inputB, const GearVelocity& inputV, const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV) {
	checkf(aB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));
	checkf(bB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));

	float Lambda = (inputV.RadiansPerSecond - 0.5f * aV.RadiansPerSecond - 0.5f * bV.RadiansPerSecond) /
		(inputB.inverseAngularMomentum + 0.25f * aB.inverseAngularMomentum + 0.25f * bB.inverseAngularMomentum);

	return VelocityDeltaTrio{
		-Lambda * inputB.inverseAngularMomentum,
		0.5f * Lambda * aB.inverseAngularMomentum,
		0.5f * Lambda * bB.inverseAngularMomentum
	};
}

VelocityDeltaPair SolveGearConnection(const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV, const float& gearing) {
	checkf(aB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));
	checkf(bB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));

	float Lambda = (gearing * bV.RadiansPerSecond - aV.RadiansPerSecond) / (aB.inverseAngularMomentum + bB.inverseAngularMomentum * gearing * gearing);
	return VelocityDeltaPair{
		Lambda * aB.inverseAngularMomentum,
		-Lambda * bB.inverseAngularMomentum * gearing
	};
}

VelocityDeltaPair SolveClutchConnection(const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV, const float& gearing, const float& maxTorque, const float& deltaTime) {
	checkf(aB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));
	checkf(bB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));

	float Lambda = (gearing * bV.RadiansPerSecond / deltaTime - aV.RadiansPerSecond / deltaTime) / (aB.inverseAngularMomentum + bB.inverseAngularMomentum * gearing * gearing);

	float ClampedLambda = FMath::Clamp(Lambda, -maxTorque, maxTorque);

	return VelocityDeltaPair{
		ClampedLambda * aB.inverseAngularMomentum * deltaTime,
		-ClampedLambda * bB.inverseAngularMomentum * deltaTime * gearing
	};
}

FrictionResult SolveFrictionConnection(const GearInverseMomentum& aB, GearVelocity& aV, const float& maxTorque, const float& goalVelocity, const float& deltaTime) {
	checkf(aB.inverseAngularMomentum > 0, TEXT("GearInverseMomentum must be greater than zero"));

	float Lambda = -(goalVelocity + aV.RadiansPerSecond) / (deltaTime * aB.inverseAngularMomentum);

	float ClampedLambda = FMath::Clamp(Lambda, -maxTorque, maxTorque);

	return FrictionResult{
		ClampedLambda,
		ClampedLambda * deltaTime * aB.inverseAngularMomentum
	};
}