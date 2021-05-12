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

  /// Serializes the current state of the whole episode.
  class PixelCountEventSerializer {
  public:

    struct Data {

      uint64_t pixel_count;

      MSGPACK_DEFINE_ARRAY(pixel_count)
    };

    constexpr static auto header_offset = 0u;

    static Data DeserializeRawData(const RawData &message) {
      return MsgPack::UnPack<Data>(message.begin(), message.size());
    }

    template <typename SensorT>
    static Buffer Serialize(
        const SensorT &,
        uint64_t pixel_count) {
      return MsgPack::Pack(Data{pixel_count});
    }

    static SharedPtr<SensorData> Deserialize(RawData &&data);
  };

} // namespace s11n
} // namespace sensor
} // namespace carla
