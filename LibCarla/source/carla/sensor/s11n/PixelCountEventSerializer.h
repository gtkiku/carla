// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Buffer.h"
#include "carla/Debug.h"
#include "carla/Memory.h"
#include "carla/rpc/Actor.h"
#include "carla/sensor/RawData.h"

namespace carla {
namespace sensor {

  class SensorData;

namespace s11n {

  class PixelCountEventSerializer {
  public:
    struct Data {
      uint64_t pixel_count;
    };

    constexpr static auto header_offset = 0u;

    static Data DeserializeRawData(const RawData &message) {
      return Data{*reinterpret_cast<const uint64_t *>(message.begin())};
    }

    template <typename SensorT>
    static Buffer Serialize(
        const SensorT &,
        uint64_t pixel_count) {
      Buffer buf;
      buf.copy_from(reinterpret_cast<const unsigned char *>(&pixel_count), sizeof(pixel_count));
      return buf;
    }

    static SharedPtr<SensorData> Deserialize(RawData &&data);
  };

} // namespace s11n
} // namespace sensor
} // namespace carla
