// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla.h"
#include "Carla/Sensor/SceneCaptureCameraOCL.h"

#include "Runtime/RenderCore/Public/RenderingThread.h"

FActorDefinition ASceneCaptureCameraOCL::GetSensorDefinition()
{
  constexpr bool bEnableModifyingPostProcessEffects = false;
  return UActorBlueprintFunctionLibrary::MakeCameraDefinition(
      TEXT("rgb_ocl"),
      bEnableModifyingPostProcessEffects);
}

ASceneCaptureCameraOCL::ASceneCaptureCameraOCL(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
    bEnablePostProcessingEffects = false;
    OCLman.initialize(ImageWidth, ImageHeight);
}

void ASceneCaptureCameraOCL::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ASceneCaptureCameraOCL::PostPhysTick);
    FPixelReader::OpenCLPixelsInRenderThread(*this, false, OCLman);
}
