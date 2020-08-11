#include "Vehicle4WActor.h"

#include "../../Intermediate/ProjectFiles/Vehicle4WSimulator.hpp"
#include "Components/BoxComponent.h"
#include "Components/InputComponent.h"
#include "Components/SphereComponent.h"
#include "Kismet/GameplayStatics.h"

Vector3f FVector2Eigen(FVector vec) {
    return Vector3f(vec.X, vec.Y, vec.Z);
}

Vector3f FRotator2Eigen(FRotator rot) {
    return Vector3f(rot.Pitch, rot.Yaw, rot.Roll);
}

Quaternionf FQuat2Eigen(FQuat quat_) {
    return Quaternionf(quat_.W, quat_.X, quat_.Y, quat_.Z);
}

FVector Eigen2FVector(Eigen::Vector3f vec) {
    return FVector(vec.x(), vec.y(), vec.z());
}

FRotator Eigen2FRotator(Eigen::Vector3f vec) {
    return FRotator(vec.x(), vec.y(), vec.z());
}

FQuat Eigen2FQuat(Eigen::Quaternionf quat_) {
    return FQuat(quat_.x(), quat_.y(), quat_.z(), quat_.w());
}

// Sets default values
AVehicle4WActor::AVehicle4WActor() {
    // Set this actor to call Tick() every frame.  You can turn this off to
    // improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;

    // root
    RootComp = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    RootComp->SetupAttachment(GetRootComponent());

    // body
    BodyComp = CreateDefaultSubobject<UBoxComponent>(TEXT("Body"));
    BodyComp->SetupAttachment(GetRootComponent());
    BodyComp->SetBoxExtent(FVector(150.f, 150.f, 50.f));

    // wheel
    for (int i = 0; i < 4; i++) {
        (*(WheelComp[i])) = CreateDefaultSubobject<USphereComponent>(
            FName(*(FString("Wheel") + FString::FromInt(i))));
        (*(WheelComp[i]))->SetupAttachment(GetRootComponent());
        (*(WheelComp[i]))->SetSphereRadius(20.f);
    }

    // input
    InputComp = CreateDefaultSubobject<UInputComponent>(TEXT("Input"));
    InputComponent = InputComp;
}

// Called when the game starts or when spawned
void AVehicle4WActor::BeginPlay() {
    Super::BeginPlay();

    float body_mass = 100.f, wheel_mass = 20.f;
    FVector location = GetActorLocation();
    Quaternionf quat =
        FQuat2Eigen(BodyComp->GetRelativeRotation().Quaternion());
    Vector3f linear_velocity(0.f, 0.f, 0.f);
    Vector3f angular_velocity(0.f, 0.f, 0.f);

    // body
    FTransform body_transform = BodyComp->GetComponentToWorld();
    FVector body_box_extent = BodyComp->CalcBounds(body_transform).BoxExtent;
    FVector body_relative_location = BodyComp->GetRelativeLocation();

    // wheel
    FTransform wheel_transform;
    wheel_transform = (*(WheelComp[0]))->GetComponentToWorld();
    float wheel_radius =
        (*(WheelComp[0]))->CalcBounds(wheel_transform).BoxExtent.X;
    Vector3f wheel_relative_location[4];
    for (int i = 0; i < 4; i++) {
        wheel_relative_location[i] =
            FVector2Eigen((*(WheelComp[i]))->GetRelativeLocation());
    }

    simulator = new Vehicle4WSimulator(
        body_mass, wheel_mass, FVector2Eigen(location), quat,
        FVector2Eigen(body_box_extent), wheel_radius, linear_velocity,
        angular_velocity, FVector2Eigen(body_relative_location),
        wheel_relative_location);

    // input
    UGameplayStatics::GetPlayerController(GetWorld(), 0);
    InputComponent->BindAction("MoveForward", EInputEvent::IE_Released, this,
                               &AVehicle4WActor::MoveForward);
    InputComponent->BindAction("MoveBackward", EInputEvent::IE_Released, this,
                               &AVehicle4WActor::MoveBackward);
    InputComponent->BindAction("TurnLeft", EInputEvent::IE_Released, this,
                               &AVehicle4WActor::TurnLeft);
    InputComponent->BindAction("TurnRight", EInputEvent::IE_Released, this,
                               &AVehicle4WActor::TurnRight);
}

// Called every frame
void AVehicle4WActor::Tick(float DeltaTime) {
    Super::Tick(DeltaTime);

    FHitResult Hit;
    bool bHasHit;
    FCollisionShape SphereShape =
        FCollisionShape::MakeSphere(simulator->get_wheel_radius());
    FVector location, end;
    Vector3f* hit_point_arr[4];
    for (int i = 0; i < 4; i++) {
        location = Eigen2FVector(simulator->get_wheel_location(i));
        end = location;
        end.Z -= 1.0f;
        bHasHit = GetWorld()->SweepSingleByChannel(
            Hit, location, end, GetActorRotation().Quaternion(),
            ECollisionChannel::ECC_WorldDynamic, SphereShape);

        if (bHasHit) {
            Vector3f hit_point = FVector2Eigen(Hit.ImpactPoint);
            hit_point_arr[i] = &hit_point;
        } else {
            hit_point_arr[i] = nullptr;
        }
    }

    simulator->apply(hit_point_arr, DeltaTime);

    // relative
    FVector relative_location;

    // body
    relative_location = Eigen2FVector(simulator->get_body_relative_location());
    BodyComp->SetRelativeLocation(relative_location);

    // wheel
    for (int i = 0; i < 4; i++) {
        relative_location =
            Eigen2FVector(simulator->get_wheel_relative_location(i));
        (*(WheelComp[i]))->SetRelativeLocation(relative_location);
    }

    // angular
    FQuat quat = Eigen2FQuat(simulator->get_wheel_relative_quat(0));
    RootComp->SetRelativeRotation(quat.Rotator());

    // display
    /*UE_LOG(LogTemp, Display, TEXT("Wheel Location: %s"),
            *relative_location.ToString());
    FVector vel = Eigen2FVector(simulator->get_wheel_linear_velocity(0));
    UE_LOG(LogTemp, Display, TEXT("Wheel Vel: %s"),
            *vel.ToString());
    GEngine->AddOnScreenDebugMessage(
            -1, 5.f, FColor::White,
            FString::Printf(TEXT("Wheel Location: %s"),
                    *relative_location.ToString()));*/
}

void AVehicle4WActor::MoveForward() {
    simulator->move(true);
}

void AVehicle4WActor::MoveBackward() {
    simulator->move(false);
}

void AVehicle4WActor::TurnLeft() {
    simulator->turn(true);
}

void AVehicle4WActor::TurnRight() {
    simulator->turn(false);
}
