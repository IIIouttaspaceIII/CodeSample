// Fill out your copyright notice in the Description page of Project Settings.

#include "FlecsSubsystem.h"

struct VelocityDeltaPair {
	float aDelta;
	float bDelta;
};

struct FrictionResult {
	float AppliedForce;
	float DeltaVelocity;
};

struct VelocityDeltaTrio {
	float inputDelta;
	float aDelta;
	float bDelta;
};

struct SubsystemReference {
	UFlecsSubsystem* subsystem;
};

inline VelocityDeltaTrio SolveDifferentialConnection(const GearInverseMomentum& inputB, const GearVelocity& inputV, const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV) {
	assert(aB.inverseAngularMomentum > 0);
	assert(bB.inverseAngularMomentum > 0);

	float Lambda = (inputV.RadiansPerSecond - 0.5 * aV.RadiansPerSecond - 0.5 * bV.RadiansPerSecond) /
		(inputB.inverseAngularMomentum + 0.25 * aB.inverseAngularMomentum + 0.25 * bB.inverseAngularMomentum);

	return VelocityDeltaTrio{
		-Lambda * inputB.inverseAngularMomentum,
		0.5f * Lambda * aB.inverseAngularMomentum,
		0.5f * Lambda * bB.inverseAngularMomentum
	};
}

inline VelocityDeltaPair SolveGearConnection(const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV, const float& gearing) {
	assert(aB.inverseAngularMomentum > 0);
	assert(bB.inverseAngularMomentum > 0);

	float Lambda = (gearing * bV.RadiansPerSecond - aV.RadiansPerSecond) / (aB.inverseAngularMomentum + bB.inverseAngularMomentum * gearing * gearing);
	return VelocityDeltaPair{
		Lambda * aB.inverseAngularMomentum,
		-Lambda * bB.inverseAngularMomentum * gearing
	};
}

inline VelocityDeltaPair SolveClutchConnection(const GearInverseMomentum& aB, const GearVelocity& aV, const GearInverseMomentum& bB, const GearVelocity& bV, const float& gearing, const float& maxTorque, const float& deltaTime) {
	assert(aB.inverseAngularMomentum > 0);
	assert(bB.inverseAngularMomentum > 0);

	float Lambda = (gearing * bV.RadiansPerSecond / deltaTime - aV.RadiansPerSecond / deltaTime) / (aB.inverseAngularMomentum + bB.inverseAngularMomentum * gearing * gearing);

	float ClampedLambda = FMath::Clamp(Lambda, -maxTorque, maxTorque);

	return VelocityDeltaPair{
		ClampedLambda * aB.inverseAngularMomentum * deltaTime,
		-ClampedLambda * bB.inverseAngularMomentum * deltaTime * gearing
	};
}

inline FrictionResult SolveFrictionConnection(const GearInverseMomentum& aB, GearVelocity& aV, const float& maxTorque, const float& goalVelocity, const float& deltaTime) {
	assert(aB.inverseAngularMomentum > 0);
	assert(bB.inverseAngularMomentum > 0);

	float Lambda = -(goalVelocity + aV.RadiansPerSecond) / (deltaTime * aB.inverseAngularMomentum);

	float ClampedLambda = FMath::Clamp(Lambda, -maxTorque, maxTorque);

	return FrictionResult{
		ClampedLambda,
		ClampedLambda * deltaTime * aB.inverseAngularMomentum
	};
}

inline static float GetResistanceAtAngularVelocity(const MotorResistances& resistances, const MotorRedline& redline, float throttle, float angularVelocity) {
	if (angularVelocity > redline.RadiansPerSecond) {
		throttle = 0.f; // If the angular velocity exceeds the redline, no throttle is applied
	}
	const auto AbsoluteAngularVelocity = FMath::Abs(angularVelocity);
	float TotalResistance = (resistances.FlatResistance + resistances.LinearResistance * angularVelocity + resistances.ExponentialResistance * AbsoluteAngularVelocity * AbsoluteAngularVelocity) * -FMath::Sign(angularVelocity);
	return (1 - throttle) * TotalResistance;
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

flecs::world* UFlecsSubsystem::GetEcsWorld() const { return ECSWorld; }
void UFlecsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	OnTickDelegate = FTickerDelegate::CreateUObject(this, &UFlecsSubsystem::Tick);
	OnTickHandle = FTSTicker::GetCoreTicker().AddTicker(OnTickDelegate);

	//sets title in Flecs Explorer
	char name[] = { "Minimum Viable Flecs" };
	char* argv = name;
	ECSWorld = new flecs::world(1, &argv);
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

inline FVector2D WheelSpaceDirectionToWorldSpace(const FVector2D& wheelSpaceDirection, const float carRotation, const float wheelAngle) {
	// Convert a direction in wheel space to world space
	return wheelSpaceDirection.GetRotated(carRotation + wheelAngle);
}

inline FVector2D WorldSpaceDirectionToWheelSpace(const FVector2D& worldSpaceDirection, const float carRotation, const float wheelAngle) {
	// Convert a direction in world space to wheel space
	return worldSpaceDirection.GetRotated(-carRotation - wheelAngle);
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

void UpdateVelocitiesAtPoints(flecs::iter& it, size_t, WheelContactVelocity& velocityAtPoint, const PositionOffset& positionOffset, const WheelAngle& wheelAngle) {
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

void SolveFrictionalForces(flecs::iter& it, size_t, const WheelContactVelocity& velocityAtPoint, const PositionOffset& positionOffset, const WheelRadius& wheelRadius, GearVelocity& gearVelocity, const GearInverseMomentum& gearInverseMomentum, const WheelAngle& wheelAngle) {
	//We check forwards velocity compared to the wheel's velocity
	const auto difference = FVector2D{ velocityAtPoint.Forwards - (gearVelocity.RadiansPerSecond * wheelRadius.value), velocityAtPoint.Right };

	//We apply a force in the opposite direction of the difference
	const float frictionMax = 7000.f; // Maximum friction force, adjust as needed
	const auto frictionForce = -difference.GetSafeNormal() * FMath::Clamp(difference.Size() * 1000.f, 0.f, frictionMax); // Adjust the multiplier as needed

	const auto carRotation = positionOffset.parent.get<Rotation>().value; // Get the car's rotation

	//const auto finalForce = frictionForce.GetRotated(carRotation - wheelAngle.value);
	const auto finalForce = WheelSpaceDirectionToWorldSpace(frictionForce, carRotation, wheelAngle.value); // Convert the friction force to world space

	/*
	it.world().get<SubsystemReference>().subsystem->DrawDebugVector(
		GetWheelPositionWorldSpace(positionOffset) * 100,
		WheelSpaceDirectionToWorldSpace(FVector2D{ 10000,0 }, carRotation, wheelAngle.value),
		FColor::Blue
	);

	it.world().get<SubsystemReference>().subsystem->DrawDebugVector(
		GetWheelPositionWorldSpace(positionOffset) * 100,
		WheelSpaceDirectionToWorldSpace(FVector2D{ 0,10000 }, carRotation, wheelAngle.value),
		FColor::Purple
	);

	it.world().get<SubsystemReference>().subsystem->DrawDebugVector(
		GetWheelPositionWorldSpace(positionOffset) * 100,
		finalForce * 10,
		FColor::Red
	);*/
	ApplyForceAtPoint(it.delta_time(), finalForce, FVector2D{ positionOffset.value.X, positionOffset.value.Y }, positionOffset);
	//ApplyForceAtPoint(it.delta_time(), WheelSpaceDirectionToWorldSpace(FVector2D{ 1000,0 }, carRotation, 0), FVector2D{ 0, 0.0001 }, positionOffset);

	//Now we need to apply opposite forces to the wheel's velocity delta
	const auto wheelForwardVector = GetWheelForwardVectorWorldSpace(positionOffset, wheelAngle);

	const auto wheelForwardFriction = frictionForce.X;//frictionForce.Dot(wheelForwardVector); // Friction force in the forward direction
	gearVelocity.RadiansPerSecond -= wheelForwardFriction * gearInverseMomentum.inverseAngularMomentum * wheelRadius.value * it.delta_time(); // Apply the friction force to the gear's velocity
}

void ApplyVelocityDeltas(flecs::iter& it, size_t, VelocityDelta& velocityDelta, Velocity& velocity, RotationalVelocity& rotationalVelocity) {
	velocity.value += velocityDelta.value;
	rotationalVelocity.value += velocityDelta.rotationDelta;
	velocityDelta.value = FVector2D::ZeroVector; // Reset the delta after applying
	velocityDelta.rotationDelta = 0.f; // Reset the rotation delta after applying
}

void UFlecsSubsystem::DrawDebugVector(const FVector2D& start, const FVector2D& Direction, const FColor& color) {
	// This function is a placeholder for drawing debug lines in Unreal Engine
	// You can implement it using Unreal's debug drawing functions
	const FVector start3D = FVector(start.X, start.Y, 100); // Assuming a fixed Z value for debug drawing
	DrawDebugLine(GetWorld(), start3D, start3D + FVector(Direction.X, Direction.Y, 0), color, false, -1.f, 0, 100.f);
}

void UFlecsSubsystem::InitFlecs()
{
	auto ecs = GetEcsWorld();
	if (!ecs) return;
	// Register components
	ecs->system<MotorTorque, const MotorResistances, const MotorRedline, const GearVelocity, const ThrottleInput, const MotorCurve>("UpdateAllMotorTorques")
		.each(UpdateAllMotorTorques);
	ecs->system<GearVelocityDelta, const MotorTorque, const GearInverseMomentum>("ApplyAllMotorTorques")
		.each(ApplyAllMotorTorques);
	ecs->system<GearVelocityDelta, GearVelocity>("ApplyAndClearDeltas")
		.each(ApplyAndClearDeltas);
	ecs->system<GearInverseMomentum, GearVelocityDelta, const GearVelocity, const Brake, const BrakeInput>("UpdateAllBrakes")
		.each(UpdateAllBrakes);
	ecs->system<const GearConnection>("SolveGearConnections")
		.each(SolveGearConnections);
	ecs->system<ClutchConnection, const ClutchInput, const ClutchStrength>("UpdateClutchInputs")
		.each(UpdateClutchInputs);
	ecs->system<const ClutchConnection>("SolveClutchConnections")
		.each(SolveClutchConnections);
	ecs->system<const DifferentialConnection>("SolveDifferentialConnections")
		.each(SolveDifferentialConnections);
	ecs->system<WheelContactVelocity, const PositionOffset, const WheelAngle>("UpdateVelocitiesAtPoints")
		.each(UpdateVelocitiesAtPoints);
	ecs->system<WheelContactVelocity, const PositionOffset, const WheelRadius, GearVelocity, const GearInverseMomentum, const WheelAngle>("SolveFrictionalForces")
		.each(SolveFrictionalForces);
	ecs->system<VelocityDelta, Velocity, RotationalVelocity>("ApplyVelocityDeltas")
		.each(ApplyVelocityDeltas);

	ecs->system<Position, const Velocity>("IntegratePosition")
		.each(IntegratePosition);
	ecs->system<Rotation, const RotationalVelocity>("IntegrateRotation")
		.each(IntegrateRotation);

	ecs->set<SubsystemReference>({ this });
	UE_LOG(LogTemp, Warning, TEXT("Flecs systems initialized!"));
}

FFlecsEntityHandle UFlecsSubsystem::MakeCarBody() {
	auto ecs = GetEcsWorld();
	return FFlecsEntityHandle{
		ecs->entity()
		.set<Position>({ FVector2D{ 0.f, 0.f } }) // Initial position
		.set<Rotation>({ 0.f }) // Initial rotation
		.set<Velocity>({ FVector2D{ 0.f, 0.f} }) // Initial velocity
		.set<RotationalVelocity>({ 0 }) // Initial rotational velocity
		.set<VelocityDelta>({ FVector2D{0.f, 0.f}, 0 }) // Initial velocity delta
		.set<Mass>({ 1400.f }) // Default mass
		.set<RotationalInertia>({ 100.f }) // Default rotational inertia
	};
}
void UFlecsSubsystem::AttachWheelToCar(FFlecsEntityHandle car, FFlecsEntityHandle wheel, FVector2D offset) {
	auto ecs = GetEcsWorld();
	wheel.FlecsEntityId
		.set<PositionOffset>({ offset, car.FlecsEntityId }) // Offset from the car's position
		.set<WheelContactVelocity>({ 0.f, 0.f }) // Initial velocity at the wheel attachment point'
		.set<WheelRadius>({ 0.05f })
		.set<WheelAngle>({ 0 }); // Default wheel radius
}

FVector2D UFlecsSubsystem::GetCarWorldSpace(FFlecsEntityHandle car) {
	return car.FlecsEntityId.get<Position>().value; // Placeholder for car world space position
}
FVector2D UFlecsSubsystem::GetWheelWorldSpace(FFlecsEntityHandle wheelAttachment) {
	return FVector2D{ 0.f, 0.f }; // Placeholder for car world space position
}

float UFlecsSubsystem::GetCarRotation(FFlecsEntityHandle car) {
	return car.FlecsEntityId.get<Rotation>().value; // Placeholder for car rotation in radians
}

float UFlecsSubsystem::GetWheelRotation(FFlecsEntityHandle wheelAttachment) {
	return 0; // Placeholder for car world space position
}

void UFlecsSubsystem::DestroyEntity(FFlecsEntityHandle entity) {
	if (!entity.FlecsEntityId.is_valid()) return;
	auto ecs = GetEcsWorld();
	ecs->entity(entity.FlecsEntityId).destruct();
	entity.FlecsEntityId = flecs::entity(); // Reset the handle
}

FFlecsEntityHandle UFlecsSubsystem::MakeDrivetrainMass(float AngularMomentum) {
	auto ecs = GetEcsWorld();
	return FFlecsEntityHandle{
		ecs->entity()
			.set<GearInverseMomentum>({ 1.f / AngularMomentum }) // Default inverse momentum
			.set<GearVelocity>({ 0.f })
			.set<GearVelocityDelta>({ 0.f })
	}; // Default velocity
}

void UFlecsSubsystem::AttachMotor(FFlecsEntityHandle handle, UCurveFloat* TorqueCurve) {
	handle.FlecsEntityId
		.set<MotorCurve>({ TorqueCurve })
		.set<ThrottleInput>({ 0 })
		.set<MotorTorque>({ 0 })
		.set<MotorResistances>({ 6.0, 0.2,0.0003f })
		.set<MotorRedline>({ 733 });
}

void UFlecsSubsystem::AttachBrake(FFlecsEntityHandle handle, float MaxTorque) {
	handle.FlecsEntityId
		.set<Brake>({ MaxTorque })
		.set<BrakeInput>({ 0 });
}

FFlecsEntityHandle UFlecsSubsystem::MakeGearConnection(FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle) {
	auto ecs = GetEcsWorld();
	return FFlecsEntityHandle{
		ecs->entity()
			.set<GearConnection>({
			1,
			aHandle.FlecsEntityId,
			bHandle.FlecsEntityId
				})
	};
}

FFlecsEntityHandle UFlecsSubsystem::MakeClutchConnection(FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle) {
	auto ecs = GetEcsWorld();
	return FFlecsEntityHandle{
		ecs->entity()
			.set<ClutchConnection>({
			1,
			0,
			aHandle.FlecsEntityId,
			bHandle.FlecsEntityId
				})
		.set<ClutchInput>({ 0 })
		.set<ClutchStrength>({ 100000000.f }) // Default max torque
	};
}

FFlecsEntityHandle UFlecsSubsystem::MakeDifferentialConnection(FFlecsEntityHandle inputHandle, FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle) {
	auto ecs = GetEcsWorld();
	return FFlecsEntityHandle{
		ecs->entity()
			.set<DifferentialConnection>({
			inputHandle.FlecsEntityId,
			aHandle.FlecsEntityId,
			bHandle.FlecsEntityId
				})
	};
}

float UFlecsSubsystem::GetAngularVelocity(const FFlecsEntityHandle handle) const {
	return handle.FlecsEntityId.get<GearVelocity>().RadiansPerSecond;
}

float UFlecsSubsystem::GetCarVelocity(FFlecsEntityHandle car) const {
	return car.FlecsEntityId.get<Velocity>().value.Size(); // Returns the magnitude of the velocity vector
}

float UFlecsSubsystem::GetCarAngularVelocity(FFlecsEntityHandle car) const {
	return car.FlecsEntityId.get<RotationalVelocity>().value; // Returns the angular velocity in radians per second
}

void UFlecsSubsystem::SetThrottle(FFlecsEntityHandle handle, float amount) {
	handle.FlecsEntityId.get_mut<ThrottleInput>().amount = amount;
}
void UFlecsSubsystem::SetClutch(FFlecsEntityHandle handle, float amount) {
	handle.FlecsEntityId.get_mut<ClutchInput>().amount = amount;
}
void UFlecsSubsystem::SetBrake(FFlecsEntityHandle handle, float amount) {
	handle.FlecsEntityId.get_mut<BrakeInput>().amount = amount;
}
void UFlecsSubsystem::SetGearRatio(FFlecsEntityHandle handle, float amount) {
	handle.FlecsEntityId.get_mut<ClutchConnection>().ratio = amount;
}
void UFlecsSubsystem::SetWheelAngle(FFlecsEntityHandle handle, float amount) {
	handle.FlecsEntityId.get_mut<WheelAngle>().value = amount; // Set the wheel angle
}

void UFlecsSubsystem::Deinitialize()
{
	FTSTicker::GetCoreTicker().RemoveTicker(OnTickHandle);

	if (ECSWorld)
	{
		delete ECSWorld;
		ECSWorld = nullptr;
	}

	UE_LOG(LogTemp, Warning, TEXT("UFlecsSubsystem has shut down!"));
	Super::Deinitialize();
}

bool UFlecsSubsystem::Tick(float DeltaTime)
{
	accumulator.fixedTimestep = 1.f / 300.f;
	accumulator.Add(DeltaTime);
	if (ECSWorld) {
		while (accumulator.Iterate()) {
			ECSWorld->progress(accumulator.fixedTimestep);
		}
	}
	return true;
}