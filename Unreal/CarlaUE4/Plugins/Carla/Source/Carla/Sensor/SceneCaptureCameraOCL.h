// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/PixelReader.h"
#include "Carla/Sensor/SceneCaptureSensor.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/OpenCL/OpenCLcontext.h"
#include <compiler/enable-ue4-macros.h>

#include "SceneCaptureCameraOCL.generated.h"

/// A sensor that captures images from the scene.
UCLASS()
class CARLA_API ASceneCaptureCameraOCL : public ASceneCaptureSensor
{
  GENERATED_BODY()

public:

  static FActorDefinition GetSensorDefinition();

  ASceneCaptureCameraOCL(const FObjectInitializer &ObjectInitializer);

protected:

  void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds) override;

private:

  OpenCL_Manager OCLman;
};
