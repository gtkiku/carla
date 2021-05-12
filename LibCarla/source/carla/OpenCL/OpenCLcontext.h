
#pragma once

#include <mutex>
#include <memory>

class OpenCL_Context;

class OpenCL_Manager {
    bool isValid;
    std::unique_ptr<OpenCL_Context> Context;
    std::mutex OclMutex;

public:

    OpenCL_Manager();
    ~OpenCL_Manager();

    // bpp = bits / pixel
    bool initialize(unsigned width, unsigned height, unsigned bpp = 32, unsigned platID = 0, unsigned devID = 0);
    bool processCameraFrame(unsigned char* input, unsigned long *output);

};
