// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "flecs.h"
#include "FlecsSubsystem.generated.h"

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
	float fixedTimestep = 1.f / 300.f; // Default to 300Hz
	int maxIterations = 50; // Maximum iterations for the accumulator
	Accumulator(float fixedTimestep = 1.f/300.f)
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
	float GetAngularVelocity(FFlecsEntityHandle handle) const;

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeGearConnection(FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle);

	UFUNCTION(BlueprintCallable, Category = "FLECS")
	FFlecsEntityHandle MakeClutchConnection(FFlecsEntityHandle aHandle, FFlecsEntityHandle bHandle);

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

protected:
	FTickerDelegate OnTickDelegate;
	FTSTicker::FDelegateHandle OnTickHandle;
	flecs::world* ECSWorld = nullptr;
private:
	bool Tick(float DeltaTime);

};
