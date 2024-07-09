// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/RayCastLidarOCL.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/ros2/ROS2.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"

FActorDefinition ARayCastLidarOCL::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_ocl"));
}


ARayCastLidarOCL::ARayCastLidarOCL(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

    RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
    SetSeed(Description.RandomSeed);
  }

void ARayCastLidarOCL::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastLidarOCL::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);

  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void ARayCastLidarOCL::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidarOCL::PostPhysTick);
  SimulateLidar(DeltaTime);

  auto DataStream = GetDataStream(*this);
  auto SensorTransform = DataStream.GetSensorTransform();

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto Buffer = DataStream.PopBufferFromPool();
    ctx.GenerateGridmap(LidarData._points, Buffer);
    //auto BufView = carla::BufferView::CreateFrom(std::move(Buffer));
    DataStream.SerializeAndSend(*this, std::move(Buffer));
    //DataStream.SerializeAndSend(*this, LidarData, DataStream.PopBufferFromPool());
  }
  // ROS2
#if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (ROS2->IsEnabled())
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("ROS2 Send");
    auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
    AActor* ParentActor = GetAttachParentActor();
    if (ParentActor)
    {
      FTransform LocalTransformRelativeToParent = GetActorTransform().GetRelativeTransform(ParentActor->GetActorTransform());
      ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, LocalTransformRelativeToParent, LidarData, this);
    }
    else
    {
      ROS2->ProcessDataFromLidar(DataStream.GetSensorType(), StreamId, SensorTransform, LidarData, this);
    }
  }
#endif


}

float ARayCastLidarOCL::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ARayCastLidarOCL::FDetection ARayCastLidarOCL::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  Detection.intensity = IntRec;

  return Detection;
}

void ARayCastLidarOCL::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  Super::PreprocessRays(Channels, MaxPointsPerChannel);

  for (auto ch = 0u; ch < Channels; ch++) {
    for (auto p = 0u; p < MaxPointsPerChannel; p++) {
      RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
    }
  }
}

bool ARayCastLidarOCL::PostprocessDetection(FDetection& Detection) const
{
  if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
    const auto ForwardVector = Detection.point.MakeUnitVector();
    const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
    Detection.point += Noise;
  }

  const float Intensity = Detection.intensity;
  if(Intensity > Description.DropOffIntensityLimit)
    return true;
  else
    return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}

void ARayCastLidarOCL::ComputeAndSaveDetections(const FTransform& SensorTransform) {
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
    PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();

  LidarData.ResetMemory(PointsPerChannel);

  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
    for (auto& hit : RecordedHits[idxChannel]) {
      FDetection Detection = ComputeDetection(hit, SensorTransform);
      if (PostprocessDetection(Detection))
        LidarData.WritePointSync(Detection);
      else
        PointsPerChannel[idxChannel]--;
    }
  }

  LidarData.WriteChannelCount(PointsPerChannel);
}
