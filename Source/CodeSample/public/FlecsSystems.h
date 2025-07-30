#pragma once
#include "CoreMinimal.h"
#include "Constraints1D.h"
#include "FlecsSubsystem.h"

inline static float GetResistanceAtAngularVelocity(const MotorResistances& resistances, const MotorRedline& redline, float throttle, float angularVelocity) {
	if (angularVelocity > redline.RadiansPerSecond) {
		throttle = 0.f; // If the angular velocity exceeds the redline, no throttle is applied
	}
	const auto AbsoluteAngularVelocity = FMath::Abs(angularVelocity);
	float TotalResistance = (resistances.FlatResistance + resistances.LinearResistance * angularVelocity + resistances.ExponentialResistance * AbsoluteAngularVelocity * AbsoluteAngularVelocity) * -FMath::Sign(angularVelocity);
	return (1 - throttle) * TotalResistance;
}

inline FVector2D WheelSpaceDirectionToWorldSpace(const FVector2D& wheelSpaceDirection, const float carRotation, const float wheelAngle) {
	// Convert a direction in wheel space to world space
	return wheelSpaceDirection.GetRotated(carRotation + wheelAngle);
}

inline FVector2D WorldSpaceDirectionToWheelSpace(const FVector2D& worldSpaceDirection, const float carRotation, const float wheelAngle) {
	// Convert a direction in world space to wheel space
	return worldSpaceDirection.GetRotated(-carRotation - wheelAngle);
}

inline FVector2D GetWorldSpacePositionToCarSpace(const FVector2D& position, const Rotation& rotation, const PositionOffset& positionOffset) {
	// Calculate the wheel's position in world space
	return WheelSpaceDirectionToWorldSpace(position - positionOffset.parent.get<Position>().value, rotation.value, 0) + positionOffset.value;
}

inline FVector2D GetCarSpacePositionToWorldSpace(const FVector2D& position, const Rotation& rotation, const FVector2D& positionOffset) {
	// Calculate the wheel's position in world space
	return WheelSpaceDirectionToWorldSpace(positionOffset, rotation.value, 0) + position;
}

inline FVector2D GetCarSpacePositionToWorldSpace(const FVector2D& position, const Rotation& rotation, const PositionOffset& positionOffset) {
	// Calculate the wheel's position in world space
	return GetCarSpacePositionToWorldSpace(position, rotation, positionOffset.parent.get<Position>().value);
}

inline static void UpdateAllMotorTorques(flecs::iter& it, size_t, MotorTorque& motorTorque, const MotorResistances& motorResistances, const MotorRedline& redline, const GearVelocity& gearVelocity, const ThrottleInput& throttleInput, const MotorCurve& motorCurve) {
	float resistance = GetResistanceAtAngularVelocity(motorResistances, redline, throttleInput.amount, gearVelocity.RadiansPerSecond);
	motorTorque.value = motorCurve.TorqueCurve ? (motorCurve.TorqueCurve->GetFloatValue(gearVelocity.RadiansPerSecond) * throttleInput.amount + resistance) : 0.f;
}

inline static void ApplyAllMotorTorques(flecs::iter& it, size_t, GearVelocityDelta& gearVelocityDelta, const MotorTorque& motorTorque, const GearInverseMomentum& gearInverseMomentum) {
	gearVelocityDelta.value += motorTorque.value * gearInverseMomentum.inverseAngularMomentum * it.delta_time();
}

inline static void ApplyAndClearDeltas(flecs::iter& it, size_t, GearVelocityDelta& gearVelocityDelta, GearVelocity& gearVelocity) {
	gearVelocity.ApplyDelta(gearVelocityDelta);
	gearVelocityDelta.ClearImpulse();
}

inline FVector2D GetWheelForwardVectorWorldSpace(const PositionOffset& positionOffset, const WheelAngle& wheelAngle) {
	// Calculate the forward vector of the wheel in world space
	const auto carRotation = positionOffset.parent.get<Rotation>().value; // Get the car's rotation
	return WheelSpaceDirectionToWorldSpace(FVector2D{ 1, 0 }, carRotation, wheelAngle.value);
}

inline FVector2D GetWheelPositionWorldSpace(const PositionOffset& positionOffset) {
	// Calculate the wheel's position in world space
	return positionOffset.parent.get<Position>().value + positionOffset.value.GetRotated(positionOffset.parent.get<Rotation>().value);
}


inline static void UpdateAllBrakes(flecs::iter& it, size_t, GearInverseMomentum& gearInverseMomentum, GearVelocityDelta& gearVelocityDelta, const GearVelocity& gearVelocity, const Brake& brake, const BrakeInput& brakeInput) {
	const auto deltaTime = it.delta_time();
	const auto Max = FMath::Abs(gearVelocity.RadiansPerSecond);
	const auto brakeMaxDelta = FMath::Clamp(brake.maxTorque * brakeInput.amount * deltaTime * gearInverseMomentum.inverseAngularMomentum, -Max, Max);
	gearVelocityDelta.value += brakeMaxDelta * -FMath::Sign(gearVelocity.RadiansPerSecond);
}

inline static void SolveGearConnections(flecs::iter& it, size_t, const GearConnection& gearConnection) {
	const auto& aB = gearConnection.a.get<GearInverseMomentum>();
	const auto& bB = gearConnection.b.get<GearInverseMomentum>();
	const auto& aV = gearConnection.a.get<GearVelocity>();
	const auto& bV = gearConnection.b.get<GearVelocity>();
	const VelocityDeltaPair deltas = SolveGearConnection(aB, aV, bB, bV, gearConnection.ratio);
	gearConnection.a.get_mut<GearVelocityDelta>().value += deltas.aDelta;
	gearConnection.b.get_mut<GearVelocityDelta>().value += deltas.bDelta;
}

inline static void SolveClutchConnections(flecs::iter& it, size_t, const ClutchConnection& clutchConnection) {
	const auto& aB = clutchConnection.a.get<GearInverseMomentum>();
	const auto& bB = clutchConnection.b.get<GearInverseMomentum>();
	const auto& aV = clutchConnection.a.get<GearVelocity>();
	const auto& bV = clutchConnection.b.get<GearVelocity>();
	const VelocityDeltaPair deltas = SolveClutchConnection(aB, aV, bB, bV, clutchConnection.ratio, clutchConnection.torque, it.delta_time());
	clutchConnection.a.get_mut<GearVelocityDelta>().value += deltas.aDelta;
	clutchConnection.b.get_mut<GearVelocityDelta>().value += deltas.bDelta;
}

inline void SolveDifferentialConnections(flecs::iter& it, size_t, const DifferentialConnection& clutchConnection) {
	const auto& inputB = clutchConnection.input.get<GearInverseMomentum>();
	const auto& aB = clutchConnection.a.get<GearInverseMomentum>();
	const auto& bB = clutchConnection.b.get<GearInverseMomentum>();
	const auto& inputV = clutchConnection.input.get<GearVelocity>();
	const auto& aV = clutchConnection.a.get<GearVelocity>();
	const auto& bV = clutchConnection.b.get<GearVelocity>();
	const VelocityDeltaTrio deltas = SolveDifferentialConnection(inputB, inputV, aB, aV, bB, bV);
	clutchConnection.input.get_mut<GearVelocityDelta>().value += deltas.inputDelta;
	clutchConnection.a.get_mut<GearVelocityDelta>().value += deltas.aDelta;
	clutchConnection.b.get_mut<GearVelocityDelta>().value += deltas.bDelta;
}

inline void UpdateClutchInputs(ClutchConnection& clutchConnection, const ClutchInput& clutchInput, const ClutchStrength& clutchStrength) {
	clutchConnection.torque = clutchInput.amount * clutchStrength.maxTorque;
}

inline void IntegratePosition(flecs::iter& it, size_t, Position& position, const Velocity& velocity) {
	position.value += velocity.value * it.delta_time();
}

inline void IntegrateRotation(flecs::iter& it, size_t, Rotation& rotation, const RotationalVelocity& rotationalVelocity) {
	rotation.value += rotationalVelocity.value * it.delta_time();
}

inline void UpdateVelocitiesAtPoints(flecs::iter& it, size_t, WheelContactVelocity& velocityAtPoint, const PositionOffset& positionOffset, const WheelAngle& wheelAngle) {
	const auto carVelocity = positionOffset.parent.get<Velocity>().value; // Get the car's position
	const auto carRotationalVelocity = positionOffset.parent.get<RotationalVelocity>().value; // Get the car's position
	const auto carRotation = positionOffset.parent.get<Rotation>().value; // Get the car's rotation
	//Now we can calculate the forward velocity and right velocity of each point (which is a wheel)

	const auto combinedVelocity = carVelocity + FMath::DegreesToRadians(carRotationalVelocity) * positionOffset.value.GetRotated(90 + carRotation); // Get the velocity in the world space

	//const auto wheelSpaceVelocity = -combinedVelocity.GetRotated(-carRotation + wheelAngle.value); // Get the velocity in the wheel's space
	const auto wheelSpaceVelocity = WorldSpaceDirectionToWheelSpace(combinedVelocity, carRotation, wheelAngle.value); // Get the velocity in the wheel's space

	//Now we can calculate the velocity at the point
	velocityAtPoint.Forwards = wheelSpaceVelocity.X;
	velocityAtPoint.Right = wheelSpaceVelocity.Y;
}

inline static void ApplyForceAtPoint(const float deltaTime, const FVector2D& forceWorldSpace, const FVector2D& relativePosition, const PositionOffset& positionOffset) {
	positionOffset.parent.get_mut<VelocityDelta>().value += deltaTime * forceWorldSpace / positionOffset.parent.get<Mass>().value; // Apply force to the car's velocity
	//Convert world space force to wheel space
	const auto forceWheelSpace = WorldSpaceDirectionToWheelSpace(forceWorldSpace, positionOffset.parent.get<Rotation>().value, 0);
	positionOffset.parent.get_mut<VelocityDelta>().rotationDelta += deltaTime * FVector2D::CrossProduct(relativePosition, forceWheelSpace) / positionOffset.parent.get<RotationalInertia>().value; // Apply torque to the car's rotational velocity
}

inline static void ApplyImpulseAtPoint(const FVector2D& forceWorldSpace, const FVector2D& relativePosition, const Rotation& rotation, Velocity& velocity, RotationalVelocity& rotationalVelocity, const Mass& mass, const RotationalInertia& rotationalInertia) {
	velocity.value += forceWorldSpace / mass.value; // Apply force to the car's velocity
	//Convert world space force to wheel space
	const auto forceWheelSpace = WorldSpaceDirectionToWheelSpace(forceWorldSpace, rotation.value, 0);
	rotationalVelocity.value += FVector2D::CrossProduct(relativePosition, forceWheelSpace) / rotationalInertia.value; // Apply torque to the car's rotational velocity
}

inline void SolveFrictionalForces(flecs::iter& it, size_t, const WheelContactVelocity& velocityAtPoint, const PositionOffset& positionOffset, const WheelRadius& wheelRadius, GearVelocity& gearVelocity, const GearInverseMomentum& gearInverseMomentum, const WheelAngle& wheelAngle) {
	//We check forwards velocity compared to the wheel's velocity
	const auto difference = FVector2D{ velocityAtPoint.Forwards - (gearVelocity.RadiansPerSecond * wheelRadius.value), velocityAtPoint.Right };

	//We apply a force in the opposite direction of the difference
	const float frictionMax = 7000.f; // Maximum friction force, adjust as needed
	const float engangementSoftness = 1000.f;
	const auto frictionForce = -difference.GetSafeNormal() * FMath::Clamp(difference.Size() * engangementSoftness, 0.f, frictionMax); // Adjust the multiplier as needed

	const auto carRotation = positionOffset.parent.get<Rotation>().value; // Get the car's rotation

	const auto finalForce = WheelSpaceDirectionToWorldSpace(frictionForce, carRotation, wheelAngle.value); // Convert the friction force to world space

	ApplyForceAtPoint(it.delta_time(), finalForce, FVector2D{ positionOffset.value.X, positionOffset.value.Y }, positionOffset);

	//Now we need to apply opposite forces to the wheel's velocity delta
	const auto wheelForwardVector = GetWheelForwardVectorWorldSpace(positionOffset, wheelAngle);

	const auto wheelForwardFriction = frictionForce.X;// Friction force in the forward direction
	gearVelocity.RadiansPerSecond -= wheelForwardFriction * gearInverseMomentum.inverseAngularMomentum * wheelRadius.value * it.delta_time(); // Apply the friction force to the gear's velocity
}

inline void ApplyVelocityDeltas(flecs::iter& it, size_t, VelocityDelta& velocityDelta, Velocity& velocity, RotationalVelocity& rotationalVelocity) {
	velocity.value += velocityDelta.value;
	rotationalVelocity.value += velocityDelta.rotationDelta;
	velocityDelta.value = FVector2D::ZeroVector; // Reset the delta after applying
	velocityDelta.rotationDelta = 0.f; // Reset the rotation delta after applying
}

struct CollisionInfo {
	int corner;
	double penetration;
};


inline static void AddBoundsCollisions(
	flecs::iter& it, size_t,
	Position& position,
	Velocity& velocity,
	Rotation& rotation,
	RotationalVelocity& rotationalVelocity,
	const BoxExtents& boxExtents,
	const RotationalInertia& rotationalInertia,
	const Mass& mass) {
	FVector2D halfExtents(boxExtents.width / 2, boxExtents.height / 2);
	FVector2D corners[4] = {
		{-halfExtents.X, -halfExtents.Y},
		{ halfExtents.X, -halfExtents.Y},
		{ halfExtents.X,  halfExtents.Y},
		{-halfExtents.X,  halfExtents.Y}
	};
	const auto restitution = 0.5f; // Coefficient of restitution for collision response
	FVector2D planes[4] = {
	{-100, -100},
	{ 100, 100},
	{ -100,  -100},
	{100,  100}
	};
	FVector2D planeNormals[4] = {
		{0, 1},
		{ 0, -1},
		{ 1,  0},
		{-1,  0}
	};
	for (int i = 0; i < 4; i++) {
		const auto planePoint = planes[i]; // The point in world space where the box is located
		const auto planeNormal = planeNormals[i]; // Normal of the collision plane, assuming the box is axis-aligned
		for (const FVector2D& localCorner : corners) {
			FVector2D worldCorner = GetCarSpacePositionToWorldSpace(position.value, rotation, localCorner);
			float distance = (worldCorner - planePoint).Dot(planeNormal);
			if (distance < 0.0f) {
				
				// Collision detected
				FVector2D r = worldCorner - position.value;
				FVector2D v_contact = velocity.value + rotationalVelocity.value * localCorner.GetRotated(90);
				float v_rel = -v_contact.Dot(planeNormal);

				// Positional correction (simple version)
				float penetration = -distance;
				FVector2D correction = planeNormal * penetration;
				position.value += correction;
			}
		}
	}
}
