// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/s11n/PixelCountEventSerializer.h"


namespace carla {
namespace sensor {
namespace data {

  /// A registered detection.
  class PixelCountEvent : public SensorData  {
    using Super = SensorData;
  protected:

    using Serializer = s11n::PixelCountEventSerializer;

    friend Serializer;

    explicit PixelCountEvent(const RawData &data)
      : Super(data) {
      auto ddata = Serializer::DeserializeRawData(data);
      _pixel_count = ddata.pixel_count;
    }

  public:

    uint64_t GetPixelCount() const {
      return _pixel_count;
    }

  private:

    uint64_t _pixel_count;
  };

} // namespace data
} // namespace sensor
} // namespace carla
