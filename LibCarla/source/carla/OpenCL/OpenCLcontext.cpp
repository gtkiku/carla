

#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#undef CL_HPP_ENABLE_EXCEPTIONS
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_TARGET_OPENCL_VERSION 120
#include <CL/cl2.hpp>

#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <mutex>
#include <cmath>
#include <condition_variable>
#include <map>
#include <set>
#include <thread>
#include <unordered_map>

#include "OpenCLcontext.h"

static const char *SOURCE = R"(

#pragma OPENCL EXTENSION cl_khr_int64_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_int64_extended_atomics : enable

__kernel void process_frame(global uchar4 *input, global ulong* output) {
    size_t x = get_global_id(0);
    size_t y = get_global_id(1);
    size_t width = get_global_size(0);
    size_t height = get_global_size(1);

    uchar4 pixel = input[y * width + x];
    if (pixel.x > 50)
        atom_add(output, 1);
}

)";


class OpenCL_Context {
    cl::Platform platform;
    std::vector<cl::Device> devices;
    cl::Device GpuDev;
    cl::Context GpuContext;
    cl::CommandQueue GpuQueue;

    cl::Program GpuProgram;
    cl::Kernel GpuKernel;

    cl::Buffer InputBuffer, OutputBuffer;
    unsigned long InputBufferSize;

    std::mutex OclMutex;
    cl::NDRange Offset, Global, Local;
    bool initialized = false;

public:
    OpenCL_Context() {};

    bool isAvailable() {
      if (!initialized)
          return false;
      if (GpuDev() == nullptr)
          return false;
      bool avail = GpuDev.getInfo<CL_DEVICE_AVAILABLE>() != CL_FALSE;
      return avail;
    }

    ~OpenCL_Context() {
        if (isAvailable())
            shutdown();
    }

    bool initialize(unsigned platID, unsigned devID, unsigned width, unsigned height, unsigned bpp);
    bool processCameraFrame(unsigned char* input, unsigned long *output);

private:
    void shutdown();
};

bool OpenCL_Manager::initialize(unsigned width, unsigned height, unsigned bpp, unsigned platID, unsigned devID) {
    if (Context->initialize(platID, devID, width, height, bpp))
        isValid = true;
    else {
        //LOG_E("Exception @ OpenCL->initialize: %s\n", e.what());
        isValid = false;
    }
    return isValid;
}

bool OpenCL_Manager::processCameraFrame(unsigned char* input, unsigned long *output) {

    std::unique_lock<std::mutex> lock(OclMutex);
    if (isValid) {
        if (!Context->processCameraFrame(input, output)) {
//            LOG_E("OCL Exception @ request_next_frame_group: %s\n", e.what());
            isValid = false;
        }
    }
    return isValid;
}

OpenCL_Manager::OpenCL_Manager() : Context{std::make_unique<OpenCL_Context>()} {}
OpenCL_Manager::~OpenCL_Manager() {}



bool OpenCL_Context::initialize(unsigned platID, unsigned devID, unsigned width, unsigned height, unsigned bpp)
{
    cl_int err;
    InputBufferSize = width * height * bpp / 8;
    // Take first platform and create a context for it.
    std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    if(!all_platforms.size()) {
        //LOG_E("No OpenCL platforms available!");
        return false;
    }

    platform = all_platforms[platID];
    Offset = cl::NullRange;

    unsigned local_h = (height % 2) ? 1 : 2;
    local_h = (height % 4) ? local_h : 4;
    local_h = (height % 8) ? local_h : 8;
    local_h = (height % 16) ? local_h : 16;
    unsigned local_w = (width % 2) ? 1 : 2;
    local_w = (width % 4) ? local_w : 4;
    local_w = (width % 8) ? local_w : 8;
    local_w = (width % 16) ? local_w : 16;

    Local = cl::NDRange(local_w, local_h);
    Global = cl::NDRange(width, height);

//    LOG_I("OpenCL platform name: %s\n",
//        platform.getInfo<CL_PLATFORM_NAME>().c_str());
//    LOG_I("OpenCL platform version: %s\n",
//        platform.getInfo<CL_PLATFORM_VERSION>().c_str());
//    LOG_I("OpenCL platform vendor: %s\n",
//        platform.getInfo<CL_PLATFORM_VENDOR>().c_str());

    // Find all devices.
    platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
    if(devices.size() == 0) {
        //LOG_E("No OpenCL devices available!");
        return false;
    }

//    LOG_I("Found %lu OpenCL devices.\n", devices.size());

//    void* ptr;
//#ifdef ENABLE_COMPRESSION
//    clSetBufferCompressionPOCL_fn SetBufferComPOCL = nullptr;
//    ptr = clGetExtensionFunctionAddressForPlatform(platform(), "clSetBufferCompressionPOCL");
//    if (ptr)
//        SetBufferComPOCL = (clSetBufferCompressionPOCL_fn)ptr;
//#endif
//    ptr = clGetExtensionFunctionAddressForPlatform(platform(), "clEnqueueReadBufferContentPOCL");
//    if (ptr == NULL)
//      throw cl::Error(CL_INVALID_PLATFORM, "platform doesn't support clEnqueueReadBufferContentPOCL");
//    enqueueReadBufferPOCL = (clEnqueueReadBufferContentPOCL_fn)ptr;

    // Open a context for them.
    GpuDev = devices[devID];
    GpuContext = cl::Context(GpuDev, nullptr, nullptr, nullptr, &err);
    if (err != CL_SUCCESS)
        return false;
    GpuQueue = cl::CommandQueue(GpuContext, GpuDev, 0, &err); // , CL_QUEUE_PROFILING_ENABLE
    if (err != CL_SUCCESS)
        return false;

    GpuProgram = cl::Program{GpuContext, SOURCE, false, &err};
    if (err != CL_SUCCESS)
        return false;
    err = GpuProgram.build();
    if (err != CL_SUCCESS)
        return false;
    GpuKernel = cl::Kernel(GpuProgram, "process_frame", &err);
    if (err != CL_SUCCESS)
        return false;

    InputBuffer = cl::Buffer(GpuContext, CL_MEM_READ_WRITE, (cl::size_type)(InputBufferSize), nullptr, &err);
    if (err != CL_SUCCESS)
        return false;
    OutputBuffer = cl::Buffer(GpuContext, CL_MEM_READ_WRITE, (cl::size_type)(sizeof(unsigned long)*64), nullptr, &err);
    if (err != CL_SUCCESS)
        return false;

#ifdef ENABLE_COMPRESSION
    if (SetBufferComPOCL) {
        int r = SetBufferComPOCL(DecodedContentBuffer(), CL_COMPRESSION_VBYTE, nullptr);
        assert (r == CL_SUCCESS);
        LOG_I ("Buffer compression VBYTE enabled\n");
    }
#endif

    GpuKernel.setArg(0, InputBuffer);
    GpuKernel.setArg(1, OutputBuffer);

    initialized = true;
    return true;
}




bool OpenCL_Context::processCameraFrame(unsigned char* input, unsigned long *output) {
    if (!isAvailable()) {
        //LOG_E("Device not available");
        return false;
    }

//    LOG_I("OCL: START request_next_frame_group %u\n", FgID);
    cl_int err;
    std::unique_lock<std::mutex> lock(OclMutex);
//    auto start_time = std::chrono::steady_clock::now();

    *output = 0;
    err = GpuQueue.enqueueWriteBuffer(InputBuffer, CL_FALSE, 0, InputBufferSize, input);
    if (err != CL_SUCCESS)
        return false;
    err = GpuQueue.enqueueWriteBuffer(OutputBuffer, CL_FALSE, 0, sizeof(cl_ulong), output);
    if (err != CL_SUCCESS)
        return false;

    err = GpuQueue.enqueueNDRangeKernel(GpuKernel, Offset, Global, Local);
    if (err != CL_SUCCESS)
        return false;

    err = GpuQueue.enqueueReadBuffer(OutputBuffer, CL_TRUE, 0, sizeof(unsigned long), output);
    if (err != CL_SUCCESS)
        return false;

//    auto end_time = std::chrono::steady_clock::now();
//    std::chrono::duration<float> diff = end_time - start_time;
//    float s = diff.count() * 1000.0f;

//    LOG_I("OCL: END request_next_frame_group %u || %03.1f ms ||", FgID, s);
    return true;
}


/*
int OpenCL_Context::wait_for_event(cl::Event &e, unsigned TimeoutMS)
{
    int status;
    for (unsigned time = 0; time < TimeoutMS; time += 10) {
        LOG_I("OCL: WAIT PERIOD\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        status = e.getInfo<CL_EVENT_COMMAND_EXECUTION_STATUS>();
        if (status <= CL_COMPLETE)
            break;
    }
    return status;
}
*/


void OpenCL_Context::shutdown() {
    if (GpuQueue())
            GpuQueue.finish();
}

