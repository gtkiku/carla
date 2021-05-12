// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/sensor/data/PixelCountEvent.h"
#include "carla/sensor/s11n/PixelCountEventSerializer.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> PixelCountEventSerializer::Deserialize(RawData &&data) {
    return SharedPtr<SensorData>(new data::PixelCountEvent(std::move(data)));
  }

} // namespace s11n
} // namespace sensor
} // namespace carla
