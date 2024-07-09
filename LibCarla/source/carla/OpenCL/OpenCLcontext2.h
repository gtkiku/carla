#pragma once

#include <mutex>
#include <memory>
#include <carla/Buffer.h>
#include <carla/BufferView.h>
#include <carla/sensor/data/LidarData.h>

class OpenCL_Context2;

class OpenCL_Manager2 {
public:
    OpenCL_Manager2();
    /* equally silly, we have to define the desstructor ourselves due to the
     * default one trying to take sizeof(OpenCL_Context2) as part of
     * std::unique_ptr. Argh x 2 */
    ~OpenCL_Manager2();
    void GenerateGridmap(std::vector<float> Data, carla::Buffer& Buffer);

private:
    std::unique_ptr<OpenCL_Context2> Context;
};
