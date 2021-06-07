

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

    uchar4 px = input[y * width + x];
    if (px.x > 100)
        atom_add(output, 1);
}
)";

#define OUTPUT_SIZE (cl::size_type)(sizeof(unsigned long)*64)

#define LOG_I(...)
#define LOG_D(...)
#define LOG_E(...)

#define CHECK_CL_ERROR(EXPR, ...) \
    if (EXPR != CL_SUCCESS) { LOG_E(__VA_ARGS__); return false; }

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
    unsigned imgID = 0;

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
        LOG_E("Error in OpenCL->initialize\n");
        isValid = false;
    }
    return isValid;
}

bool OpenCL_Manager::processCameraFrame(unsigned char* input, unsigned long *output) {

    std::unique_lock<std::mutex> lock(OclMutex);
    if (isValid) {
        if (!Context->processCameraFrame(input, output)) {
            LOG_E("Error in OpenCL->processCameraFrame\n");
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
        LOG_E("No OpenCL platforms available!\n");
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

    // Find all devices.
    platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
    if(devices.size() == 0) {
        LOG_E("No OpenCL devices available!\n");
        return false;
    }

    LOG_I("OpenCL platform name: %s\n",  platform.getInfo<CL_PLATFORM_NAME>().c_str());
    LOG_I("OpenCL platform version: %s\n", platform.getInfo<CL_PLATFORM_VERSION>().c_str());
    LOG_I("OpenCL platform vendor: %s\n",  platform.getInfo<CL_PLATFORM_VENDOR>().c_str());
    LOG_I("Found %lu OpenCL devices.\n", devices.size());

    void* ptr;
#ifdef ENABLE_COMPRESSION
    clSetBufferCompressionPOCL_fn SetBufferComPOCL = nullptr;
    ptr = clGetExtensionFunctionAddressForPlatform(platform(), "clSetBufferCompressionPOCL");
    if (ptr)
        SetBufferComPOCL = (clSetBufferCompressionPOCL_fn)ptr;
#endif
    ptr = clGetExtensionFunctionAddressForPlatform(platform(), "clEnqueueReadBufferContentPOCL");
    if (ptr == nullptr)
      LOG_I("platform doesn't support clEnqueueReadBufferContentPOCL\n");
    else
//    enqueueReadBufferPOCL = (clEnqueueReadBufferContentPOCL_fn)ptr;

    // Open a context for them.
    GpuDev = devices[devID];
    GpuContext = cl::Context(GpuDev, nullptr, nullptr, nullptr, &err);
    CHECK_CL_ERROR(err, "Context creation failed\n");
    GpuQueue = cl::CommandQueue(GpuContext, GpuDev, 0, &err); // , CL_QUEUE_PROFILING_ENABLE
    CHECK_CL_ERROR(err, "CmdQueue creation failed\n");

    GpuProgram = cl::Program{GpuContext, SOURCE, false, &err};
    CHECK_CL_ERROR(err, "Program creation failed\n");
    err = GpuProgram.build();
    CHECK_CL_ERROR(err, "Program build failed\n");
    GpuKernel = cl::Kernel(GpuProgram, "process_frame", &err);
    CHECK_CL_ERROR(err, "Kernel creation failed\n");

    InputBuffer = cl::Buffer(GpuContext, CL_MEM_READ_WRITE, (cl::size_type)(InputBufferSize), nullptr, &err);
    CHECK_CL_ERROR(err, "Input buffer creation failed\n");
    OutputBuffer = cl::Buffer(GpuContext, CL_MEM_READ_WRITE, OUTPUT_SIZE, nullptr, &err);
    CHECK_CL_ERROR(err, "Output buffer creation failed\n");

#ifdef ENABLE_COMPRESSION
    if (SetBufferComPOCL) {
        int r = SetBufferComPOCL(InputBuffer(), CL_COMPRESSION_VBYTE, nullptr);
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
        LOG_E("Device not available");
        return false;
    }

#ifdef TIMING
    LOG_D("OpenCL: start processCameraFrame\n");
    std::unique_lock<std::mutex> lock(OclMutex);
    auto start_time = std::chrono::steady_clock::now();
#endif

    cl_int err;
    err = GpuQueue.enqueueWriteBuffer(InputBuffer, CL_FALSE, 0, InputBufferSize, input);
    if (err != CL_SUCCESS)
        return false;
    *output = 0;
    err = GpuQueue.enqueueWriteBuffer(OutputBuffer, CL_FALSE, 0, sizeof(cl_ulong), output);
    if (err != CL_SUCCESS)
        return false;

    err = GpuQueue.enqueueNDRangeKernel(GpuKernel, Offset, Global, Local);
    if (err != CL_SUCCESS)
        return false;

    err = GpuQueue.enqueueReadBuffer(OutputBuffer, CL_TRUE, 0, sizeof(unsigned long), output);
    if (err != CL_SUCCESS)
        return false;

#ifdef DUMP_FRAMES
    char filename[1024];
    std::snprintf(filename, 1024, "/tmp/carla_%u_%zu.raw", imgID, *output);
    FILE* outfile = std::fopen(filename, "w");
    std::fwrite(input, 1, InputBufferSize, outfile);
    std::fclose(outfile);
    ++imgID;
#endif

#ifdef TIMING
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<float> diff = end_time - start_time;
    float s = diff.count() * 1000.0f;
    LOG_D("OpenCL: end processCameraFrame: %03.1f ms\n", s);
#endif
    return true;
}


void OpenCL_Context::shutdown() {
    if (GpuQueue())
            GpuQueue.finish();
}

