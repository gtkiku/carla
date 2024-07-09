// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/data/Array.h"
#include "carla/sensor/s11n/GridmapEventSerializer.h"


namespace carla {
namespace sensor {
namespace data {

  /// A registered detection.
  class GridmapEvent : public Array<float>  {
    using Super = Array<float>;
  protected:

    using Serializer = s11n::GridmapEventSerializer;

    friend Serializer;

    explicit GridmapEvent(RawData &&data)
      : Super(Serializer::header_offset, std::move(data)) {
        DEBUG_ASSERT(GetWidth() * GetHeight() == Super::size())
    }

  private:
    const auto &GetHeader() const {
      return Serializer::DeserializeHeader(Super::GetRawData());
    }

  public:
    auto GetWidth() const {
      return GetHeader().width;
    }

    auto GetHeight() const {
      return GetHeader().height;
    }

    auto GetSize() const {
      return Super::size();
    }

  };

} // namespace data
} // namespace sensor
} // namespace carla
