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
  class GridmapEventSerializer {
  public:

    struct GridmapHeader {
      uint32_t width;
      uint32_t height;
    };

    constexpr static auto header_offset = sizeof(GridmapHeader);

    static const GridmapHeader &DeserializeHeader(const RawData &data) {
      return *reinterpret_cast<const GridmapHeader *>(data.begin());
    }

    template <typename SensorT>
    static Buffer Serialize(
        const SensorT &,
        Buffer &&bitmap) {
      DEBUG_ASSERT(bitmap.size() > sizeof(GridmapHeader));

      /** @todo query gridmap sensor for these values */
      GridmapHeader header = {
        100,
        100
      };

      auto Buffer = carla::Buffer();
      /* leave enough room for our header at the front of the buffer */
      Buffer.copy_from(sizeof(header), bitmap);
      std::memcpy(Buffer.data(), reinterpret_cast<const void *>(&header), sizeof(header));
      return Buffer;
    }

    static SharedPtr<SensorData> Deserialize(RawData &&data);
  };

} // namespace s11n
} // namespace sensor
} // namespace carla
