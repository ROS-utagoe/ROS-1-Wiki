###### tags: `ROS` `Unreal Engine` `ROSIntegration`

Moving Actor Forward and Rotate with ROS
===
# Robot Steering (ROS)
We'll use **Robot Steering** tool of **rqt** to control an object.

Run `$ rqt` and then **rqt** window will pop up.
Select `Plugins > Robot Tools > Robot Steering`.
Set the Topic `/cmd_vel`.

<img src=https://i.imgur.com/1xBjmUq.png width=500/>

Robot Steering publishes `geometry_msgs/Twist` message ([ROS wiki](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)).

`Twist` consists of two `geometry_msgs/Vector3` messages.

    geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
    geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z

Robot Steering publishes only `linear.x` and `angular.z`.

# Robot (UE)
We use [delegate](https://docs.unrealengine.com/en-US/Programming/UnrealArchitecture/Delegates/index.html) to translate values.

#### Robot.h
```cpp
// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Runtime/Engine/Classes/Engine/StaticMeshActor.h"

#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/geometry_msgs/Twist.h"

#include "ROSIntegration/Public/std_msgs/StdMsgsString.h"


#include "Robot.generated.h"

// Delegates
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FUpdatePositionDelegate, const FVector, Linear, const FVector, Angular);


UCLASS()
class ROS_PROJECT2_API ARobot : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARobot();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	UFUNCTION(BlueprintCallable, Category = "ROSBridgeSample")
		void UpdatePosition(FVector Linear, FVector Angular);

	// Delegate
	UPROPERTY(BlueprintAssignable, Category = "ROSBridgeSample")
		FUpdatePositionDelegate OnUpdatePositionDelegate;
	

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

private:
	UTopic* Topic;	
	
};
```
#### Robot.cpp
```cpp
// Fill out your copyright notice in the Description page of Project Settings.


#include "Robot.h"


// Sets default values
ARobot::ARobot()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ARobot::BeginPlay()
{
	Super::BeginPlay();

	// Attach delegate
	OnUpdatePositionDelegate.AddDynamic(this, &ARobot::UpdatePosition);

}

// Called every frame
void ARobot::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Subscribe Topic Settings
	Topic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	Topic->Init(rosinst->ROSIntegrationCore, TEXT("/cmd_vel"), TEXT("geometry_msgs/Twist"));

	// Create a std::function callback object
	std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [&](TSharedPtr<FROSBaseMsg> msg) -> void
	{
		auto Concrete = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(msg);
		if (Concrete.IsValid())
		{
			FVector Linear, Angular;
			Linear.X = (Concrete->linear).x;
			Angular.Z = (Concrete->angular).z;

			OnUpdatePositionDelegate.Broadcast(Linear, Angular);
		}
		return;
	};
	
	// Subscribe to the topic
	Topic->Subscribe(SubscribeCallback);

}

void ARobot::UpdatePosition(FVector Linear, FVector Angular)
{
	FVector NewLocation = this->GetActorLocation();
	FRotator NewRotation = this->GetActorRotation();
	NewLocation += GetActorForwardVector() * Linear.X;
	NewRotation.Yaw += Angular.Z;
    
	UE_LOG(LogTemp, Log, TEXT("Linear: X: %lf"), (Linear.X));
	UE_LOG(LogTemp, Log, TEXT("Angular: Z: %lf"), (Angular.Z));
    
	this->SetActorLocationAndRotation(NewLocation, NewRotation, false, 0, ETeleportType::None);
}
```

`UpdatePosition()` is the main function to move the actor.
It gets `Linear` and `Angular` of `FVector` from the delegate and move the actor by `SetActorLocationAndRotation()` function.


