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
  if(ReadyToCapture)
  {
    ReadyToCapture = false;

    auto Stream = GetDataStream(*this);
    auto Input = Stream.PopBufferFromPool();
    Input.reset(ImageWidth * ImageHeight * 4);

    FPixelReader::GetPixelsInRenderThread(*this, Input);

    unsigned long Output;
    OCLman.processCameraFrame(Input.data(), &Output);

    Stream.Send(*this, Output);
  }
}
