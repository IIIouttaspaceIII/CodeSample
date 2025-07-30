#pragma once
#include "CoreMinimal.h"
#include "flecs.h"

struct Mass {
	float value = 1.f; // Default mass value
};

struct RotationalInertia {
	float value = 1.f; // Default rotational inertia
};

struct Position {
	FVector2D value;
};

struct Rotation {
	float value = 0.f; // In radians
};

struct WheelRadius {
	float value = 1.f; // Default wheel radius
};

struct WheelAngle {
	float value = 0.f; // Angle in radians
};

struct Velocity {
	FVector2D value;
};

struct VelocityDelta {
	FVector2D value;
	float rotationDelta = 0.f; // Rotation delta in radians
};

struct PositionOffset {
	FVector2D value = FVector2D::ZeroVector; // Offset from the parent entity's position
	flecs::entity parent; // Parent entity for the offset
};

struct WheelContactVelocity {
	float Forwards;
	float Right;
};

struct RotationalVelocity {
	float value = 0.f; // In radians per second
};

struct GearMomentum {
	float value = 1.f;
};
struct GearVelocityDelta {
	float value = 0.f;
	inline void ClearImpulse() { value = 0.f; }
};
struct GearVelocity {
	float RadiansPerSecond = 0.f;
	inline void ApplyDelta(const struct GearVelocityDelta& delta) {
		RadiansPerSecond += delta.value;
	}
};
struct GearInverseMomentum {
	float inverseAngularMomentum = 1.f;
};

struct MotorResistances {
	float FlatResistance = 100000.0;
	float LinearResistance = 30;
	float ExponentialResistance = 0.03f;
};

struct MotorRedline {
	float RadiansPerSecond = 1.f; // Maximum RPM in radians per second
};

struct MotorCurve {
	UCurveFloat* TorqueCurve = nullptr;
};

struct MotorTorque {
	float value = 0.f;
};

struct ThrottleInput {
	float amount = 0.f;
};

struct BrakeInput {
	float amount = 0.f;
};

struct Brake {
	float maxTorque = 1.f;
};

struct GearConnection {
	float ratio = 1.f;
	flecs::entity a;
	flecs::entity b;
};

struct ClutchInput {
	float amount = 0.f; // 0 to 1, where 1 is fully engaged
};

struct ClutchStrength {
	float maxTorque = 1.f; // Maximum torque the clutch can handle
};

struct ClutchConnection {
	float ratio = 1.f;
	float torque = 0.f; // Torque applied by the clutch
	flecs::entity a;
	flecs::entity b;
};

struct DifferentialConnection {
	flecs::entity input;
	flecs::entity a;
	flecs::entity b;
};

struct BoxExtents {
	float width = 1.f;
	float height = 1.f;
};