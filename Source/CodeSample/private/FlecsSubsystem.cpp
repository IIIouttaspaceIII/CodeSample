// Fill out your copyright notice in the Description page of Project Settings.

#include "FlecsSubsystem.h"
#include "FlecsSystems.h"
#include "FlecsStructs.h"

struct SubsystemReference {
	UFlecsSubsystem* subsystem;
};

void UFlecsSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	OnTickDelegate = FTickerDelegate::CreateUObject(this, &UFlecsSubsystem::Tick);
	OnTickHandle = FTSTicker::GetCoreTicker().AddTicker(OnTickDelegate);

	//sets title in Flecs Explorer
	char name[] = { "Minimum Viable Flecs" };
	char* argv = name;
	ECSWorld = new flecs::world(1, &argv);
}

void UFlecsSubsystem::InitFlecs()
{
	auto ecs = GetEcsWorld();
	checkf(ecs, TEXT("Flecs world is not initialized!"));
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
	accumulator.Add(DeltaTime);
	if (ECSWorld) {
		while (accumulator.Iterate()) {
			ECSWorld->progress(accumulator.fixedTimestep);
		}
	}
	return true;
}

flecs::world* UFlecsSubsystem::GetEcsWorld() const { return ECSWorld; }

void UFlecsSubsystem::DrawDebugVector(const FVector2D& start, const FVector2D& Direction, const FColor& color) {
	// This function is a placeholder for drawing debug lines in Unreal Engine
	// You can implement it using Unreal's debug drawing functions
	const float defaultRenderHeight = 100.f;
	const FVector start3D = FVector(start.X, start.Y, defaultRenderHeight); // Assuming a fixed Z value for debug drawing
	DrawDebugLine(GetWorld(), start3D, start3D + FVector(Direction.X, Direction.Y, 0), color, false, -1.f, 0, 100.f);
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
		.set<BoxExtents>({ 1.f, 0.5f }) // Default box extents
	};
}
void UFlecsSubsystem::AttachWheelToCar(FFlecsEntityHandle car, FFlecsEntityHandle wheel, FVector2D offset) {
	auto ecs = GetEcsWorld();
	wheel.FlecsEntityId
		.set<PositionOffset>({ offset, car.FlecsEntityId }) // Offset from the car's position
		.set<WheelContactVelocity>({ 0.f, 0.f }) // Initial velocity at the wheel attachment point'
		.set<WheelRadius>({ 0.05f })
		.set<WheelAngle>({ 0 });
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
	};
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
	handle.FlecsEntityId.get_mut<WheelAngle>().value = amount;
}