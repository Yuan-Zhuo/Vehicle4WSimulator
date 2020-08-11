// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
// #include "../../Intermediate/ProjectFiles/PhysicsSim.hpp"
#include "../../Intermediate/ProjectFiles/Vehicle4WSimulator.hpp"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
// last
#include "Vehicle4WActor.generated.h"

UCLASS()
class VEHICLE4WSIMULATOR_API AVehicle4WActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AVehicle4WActor();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void MoveForward();
	void MoveBackward();
	void TurnLeft();
	void TurnRight();

private:
	UPROPERTY(VisibleAnywhere)
		USceneComponent * RootComp;

	UPROPERTY(VisibleAnywhere)
		UBoxComponent * BodyComp;

	UPROPERTY(VisibleAnywhere)
		USphereComponent * WheelComp0;

	UPROPERTY(VisibleAnywhere)
		USphereComponent * WheelComp1;

	UPROPERTY(VisibleAnywhere)
		USphereComponent * WheelComp2;

	UPROPERTY(VisibleAnywhere)
		USphereComponent * WheelComp3;

	UPROPERTY(VisibleAnywhere)
		UInputComponent * InputComp;

	USphereComponent ** WheelComp[4] = { &WheelComp0, &WheelComp1, &WheelComp2, &WheelComp3 };

	Vehicle4WSimulator* simulator;
};
