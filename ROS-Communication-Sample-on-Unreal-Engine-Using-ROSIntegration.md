###### tags: `ROS` `Unreal Engine` `ROSIntegration`

ROS Communication Sample on Unreal Engine Using ROSIntegration
===

# Preparation
## Install Unreal Engine on Ubuntu
To install **Unreal Engine (UE)** on Ubuntu, follow [official tutorial](https://docs.unrealengine.com/en-US/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html).

If you have a probrem that UE constantly freezes, [this solution](https://qiita.com/deltaMASH/items/fac548203749c2a73641) might help you.
Change the `TargetedRHIs` from `SF_VULKAN_SM5` to `GLSL_430`.
`;` is used to comment out.

```
[/Script/LinuxTargetPlatform.LinuxTargetSettings]
; When neither -vulkan nor -opengl4 are passed on the commandline, the engine will default to the first targeted RHI.
; +TargetRHIs=SF_VULKAN_SM5 ;; comment out 

; OpenGL4 is deprecated, you can comment this back in to add it to your targeted RHI list
+TargetedRHIs=GLSL_430 ;; uncomment this line
```
Then **rebuild**.

## ROSIntegration
To set up [ROSIntegration](https://github.com/code-iai/ROSIntegration) on Unreal Engine, follow README.

## ROS

You need `ROSBridge`  which is also used in **ROS#**.
    
    $ sudo apt-get install ros-melodic-rosbridge-suite
    
In addtion, **ROSIntegration** uses **BSON** which is included in [PyMongo](https://api.mongodb.com/python/3.0.3/index.html) package.
Install PyMongo via:

    $ pip install pymongo
    
### rosbridge
Make sure to run this command correctly.

    $ roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True

    
# Publish/Subscribe to Topic (UE)
To test simple Publish/Subscribe communication with ROS, create a new **C++ Actor**.
Let's create **SamplePubliser** and **SampleSubscriber**.

## Publisher
In the header, add these lines ***above*** `#include "SamplePublisher.generated.h"` because `generated.h` must be included last.

In the main script, add these scripts into `BeginPlay()` method.

#### SamplePublisher.h
```cpp
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/StdMsgsString.h"
```
#### SamplePublisher.cpp
```cpp
// Initialize a topic
UTopic *ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/chatter"), TEXT("std_msgs/String"));

// (Optional) Advertise the topic
ExampleTopic->Advertise();

// Publish a string to the topic
TSharedPtr<ROSMessages::std_msgs::String> StringMessage(new ROSMessages::std_msgs::String("This is an example"));
ExampleTopic->Publish(StringMessage);
```

## Subscriber
To test if UE actually subscribes the topic, **SampleSubscriber** send the message it subscribes to `UE_LOG`.

#### SampleSubscriber.h
The same to **SamplePublisher.h**
```cpp
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/StdMsgsString.h"
```
#### SampleSubscriber.cpp
```cpp
#include "SampleSubscriber.h"

// Sets default values
ASampleSubscriber::ASampleSubscriber()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASampleSubscriber::BeginPlay()
{
	Super::BeginPlay();

}

// Create a std::function callback object
std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
{
    auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
    if (Concrete.IsValid())
    {
        UE_LOG(LogTemp, Log, TEXT("Incoming string was: %s"), (*(Concrete->_Data)));
    }
    return;
};

// Called every frame
void ASampleSubscriber::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	// Initialize a topic
	UTopic *ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/chatter"), TEXT("std_msgs/String"));
	// Subscribe to the topic
	ExampleTopic->Subscribe(SubscribeCallback);
}


```



# Test
To test if the messeges actually publish/subscribe via ROS, we use **talker** and **listener** introduced in [ROS wiki](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).
## UE to ROS (Publish message to ROS)
We use **SamplePublisher (UE)** and **listener (ROS)**.

To start with, launch `ros_bridge` first:

    $ roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
    
Then, run **listener** on ROS.

    $ rosrun <your package> talker

Next, place the Actor **SamplePublisher** in your UE field, then **Play**.
You'll see 

    [INFO] [1588662504.536355639]: I heard: [This is an example]

in the ROS console.

## ROS to UE (Subscribe message of ROS)
We use **SampleSubscriber (UE)** and **talker (ROS)**.

Run **talker** on ROS.

    $ rosrun <your package> lister

Next, place the Actor **SampleSubsecriber** in your UE field, then **Play**.

You'll see 

    LogTemp: Incoming string was: hello world 104
    
in the UE console (or output log).