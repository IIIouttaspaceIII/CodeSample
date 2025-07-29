// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "flecs.h"
#include "FlecsSubsystem.generated.h"

struct FlecsTransform
{
	FTransform Value;
};

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

USTRUCT(BlueprintType)
struct FFlecsEntityHandle
{
	GENERATED_USTRUCT_BODY()
	FFlecsEntityHandle() {}
	FFlecsEntityHandle(flecs::entity inId) :
		FlecsEntityId(inId) {
	}
	flecs::entity FlecsEntityId;
};

struct Accumulator
{
	float accumulatedTime = 0.f;
	float fixedTimestep = 1.f / 500.f; // Default to 500Hz
	int maxIterations = 50; // Maximum iterations for the accumulator
	Accumulator(float fixedTimestep = 1.f/500.f)
		: accumulatedTime(0.f), fixedTimestep(fixedTimestep) {
	}
	inline void Add(float deltaTime) {
		accumulatedTime += deltaTime;
		accumulatedTime = FMath::Clamp(accumulatedTime, 0.f, fixedTimestep * maxIterations);
	}
	inline bool IsReady() const {
		return accumulatedTime >= fixedTimestep;
	}
	inline void Reset() {
		accumulatedTime = 0.f;
	}
	inline void Subtract() {
		accumulatedTime -= fixedTimestep;
	}
	inline bool Iterate() {
		if (IsReady()) {
			Subtract();
			return true;
		}
		return false;
	}
};

/**
 * 
 */
UCLASS()
class CODESAMPLE_API UFlecsSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()
public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	flecs::world* GetEcsWorld() const;

	Accumulator accumulator;

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void InitFlecs();

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeDrivetrainMass(float AngularMomentum);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void DestroyEntity(FFlecsEntityHandle entity);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void AttachMotor(FFlecsEntityHandle handle, UCurveFloat* TorqueCurve);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void AttachBrake(FFlecsEntityHandle handle, float MaxTorque);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeDifferentialConnection(FFlecsEntityHandle inputHandle, FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeCarBody();

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void AttachWheelToCar(FFlecsEntityHandle car, FFlecsEntityHandle wheel, FVector2D offset);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FVector2D GetCarWorldSpace(FFlecsEntityHandle car);
	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FVector2D GetWheelWorldSpace(FFlecsEntityHandle wheelAttachment);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	float GetCarRotation(FFlecsEntityHandle car);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	float GetCarVelocity(FFlecsEntityHandle car) const;
	UFUNCTION(BlueprintCallable, Category = "FLECS")
	float GetCarAngularVelocity(FFlecsEntityHandle car) const;

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	float GetWheelRotation(FFlecsEntityHandle wheelAttachment);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeGearConnection(FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle);


	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeClutchConnection(FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	float GetAngularVelocity(FFlecsEntityHandle handle) const;

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void SetThrottle(FFlecsEntityHandle handle, float amount);
	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void SetClutch(FFlecsEntityHandle handle, float amount);
	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void SetGearRatio(FFlecsEntityHandle handle, float amount);
	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void SetBrake(FFlecsEntityHandle handle, float amount);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	void SetWheelAngle(FFlecsEntityHandle handle, float amount);

	void DrawDebugVector(const FVector2D& start, const FVector2D& end, const FColor& color = FColor::Red);
	/*
	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FVector2D GetWorldPosition() const;*/

	//void CreatePlayerCharacter();
protected:
	FTickerDelegate OnTickDelegate;
	FTSTicker::FDelegateHandle OnTickHandle;
	flecs::world* ECSWorld = nullptr;
private:
	bool Tick(float DeltaTime);

};
