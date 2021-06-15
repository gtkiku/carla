

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

static const char *PX_COUNT_SOURCE = R"(

#pragma OPENCL EXTENSION cl_khr_int64_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_int64_extended_atomics : enable

__kernel void process_frame(global const uchar4 *input, global ulong* output) {
    size_t x = get_global_id(0);
    size_t y = get_global_id(1);
    size_t width = get_global_size(0);
    size_t height = get_global_size(1);

    uchar4 px = input[y * width + x];
    if (px.x > 100)
        atom_add(output, 1);
}
)";

static const char *DOWNSAMPLE_SOURCE = R"(

#pragma OPENCL EXTENSION cl_khr_int64_base_atomics : enable
#pragma OPENCL EXTENSION cl_khr_int64_extended_atomics : enable

__kernel void downsample_image(global const uchar4 *input, global uchar4* output) {
    size_t x = get_global_id(0);
    size_t y = get_global_id(1);
    size_t width = get_global_size(0);
    size_t height = get_global_size(1);

/*
    uint4 sum = input[y * width + x]
              + input[y * width + x + 1]
              + input[(y+1) * width + x]
              + input[(y+1) * width + x + 1];
    output[y * (width/2) + x] = convert_uchar4(sum / 4);
*/
   unsigned out_idx = y + x*width;

   const unsigned factor = 2;
   width *= factor;
   height *= factor;
   unsigned y1, y2, x1, x2;
   y1=y*factor;
   y2=y1+1;
   x1=x*factor;
   x2=x1+1;

   uchar4 q11, q12, q21, q22;
   q11 = input[y1 * width + x1];
   q12 = input[y1 * width + x2];
   q21 = input[y2 * width + x1];
   q22 = input[y2 * width + x2];
   uint4 sum = convert_uint4(q11) + convert_uint4(q12) + convert_uint4(q21) + convert_uint4(q22);
   output[out_idx] = convert_uchar4(sum/4);
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
    cl::Context ClContext;

    cl::Device GpuDev;
    cl::CommandQueue GpuQueue;
    cl::Program GpuProgram;
    cl::Kernel GpuKernel;

    cl::Device FpgaDev;
    cl::CommandQueue FpgaQueue;
    cl::Program FpgaProgram;
    cl::Kernel FpgaKernel;

    cl::Buffer GpuInputBuffer, InterBuffer, FpgaOutputBuffer;
    unsigned long InputBufferSize;

    std::mutex OclMutex;
    cl::NDRange Offset, Global, Local;
    bool initialized = false;
    unsigned imgID = 0;

public:
    OpenCL_Context() {};

    bool isAvailable() {
      std::unique_lock<std::mutex> lock(OclMutex);
      if (!initialized)
          return false;
      if (GpuDev() == nullptr || FpgaDev() == nullptr)
          return false;
//      bool avail = GpuDev.getInfo<CL_DEVICE_AVAILABLE>() != CL_FALSE;
//      return avail;
      return true;
    }

    ~OpenCL_Context() {
        if (isAvailable())
            shutdown();
    }

    bool initialize(unsigned width, unsigned height, unsigned bpp);
    bool processCameraFrame(unsigned char* input, unsigned long *output);

private:
    void shutdown();
};

bool OpenCL_Manager::initialize(unsigned width, unsigned height, unsigned bpp) {
    if (Context->initialize(width, height, bpp))
        isValid = true;
    else {
        LOG_E("Error in OpenCL->initialize\n");
        isValid = false;
    }
    return isValid;
}

bool OpenCL_Manager::processCameraFrame(unsigned char* input, unsigned long *output) {

    if (isValid) {
        if (!Context->processCameraFrame(input, output)) {
            LOG_E("Error in OpenCL->processCameraFrame\n");
            isValid = false;
        }
    }
    if (!isValid)
        *output = 11223344;
    return isValid;
}

OpenCL_Manager::OpenCL_Manager() : Context{std::make_unique<OpenCL_Context>()} {}
OpenCL_Manager::~OpenCL_Manager() {}



bool OpenCL_Context::initialize(unsigned width, unsigned height, unsigned bpp)
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

    platform = all_platforms[0];
    Offset = cl::NullRange;

    unsigned local_h = (height % 2) ? 1 : 2;
    local_h = (height % 4) ? local_h : 4;
    local_h = (height % 8) ? local_h : 8;
    local_h = (height % 16) ? local_h : 16;
    unsigned local_w = (width % 2) ? 1 : 2;
    local_w = (width % 4) ? local_w : 4;
    local_w = (width % 8) ? local_w : 8;
    local_w = (width % 16) ? local_w : 16;

    Local = cl::NDRange(local_w/2, local_h/2);
    Global = cl::NDRange(width/2, height/2);

    // Find all devices.
    platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
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

    if (devices.size() == 1) {
        GpuDev = devices[0];
        FpgaDev = devices[0];
    } else if (devices[0].getInfo<CL_DEVICE_TYPE>() & CL_DEVICE_TYPE_GPU) {
        GpuDev = devices[0];
        FpgaDev = devices[1];
    } else {
        GpuDev = devices[1];
        FpgaDev = devices[0];
    }

    ClContext = cl::Context(devices, nullptr, nullptr, nullptr, &err);
    CHECK_CL_ERROR(err, "Context creation failed\n");

    GpuQueue = cl::CommandQueue(ClContext, GpuDev, 0, &err); // , CL_QUEUE_PROFILING_ENABLE
    CHECK_CL_ERROR(err, "CmdQueue creation failed\n");
    if (devices.size() == 1)
        FpgaQueue = GpuQueue;
    else {
        FpgaQueue = cl::CommandQueue(ClContext, FpgaDev, 0, &err); // , CL_QUEUE_PROFILING_ENABLE
        CHECK_CL_ERROR(err, "CmdQueue creation failed\n");
    }

    GpuProgram = cl::Program{ClContext, DOWNSAMPLE_SOURCE, false, &err};
    CHECK_CL_ERROR(err, "Program creation failed\n");
    err = GpuProgram.build();
    CHECK_CL_ERROR(err, "Program build failed\n");
    GpuKernel = cl::Kernel(GpuProgram, "downsample_image", &err);
    CHECK_CL_ERROR(err, "Kernel creation failed\n");

    FpgaProgram = cl::Program{ClContext, PX_COUNT_SOURCE, false, &err};
    CHECK_CL_ERROR(err, "Program creation failed\n");
    err = FpgaProgram.build();
    CHECK_CL_ERROR(err, "Program build failed\n");
    FpgaKernel = cl::Kernel(FpgaProgram, "process_frame", &err);
    CHECK_CL_ERROR(err, "Kernel creation failed\n");

    GpuInputBuffer = cl::Buffer(ClContext, CL_MEM_READ_WRITE, (cl::size_type)(InputBufferSize), nullptr, &err);
    CHECK_CL_ERROR(err, "Input buffer creation failed\n");
    InterBuffer = cl::Buffer(ClContext, CL_MEM_READ_WRITE, (cl::size_type)(InputBufferSize/4), nullptr, &err);
    CHECK_CL_ERROR(err, "Inter buffer creation failed\n");
    FpgaOutputBuffer = cl::Buffer(ClContext, CL_MEM_READ_WRITE, OUTPUT_SIZE, nullptr, &err);
    CHECK_CL_ERROR(err, "Output buffer creation failed\n");

#ifdef ENABLE_COMPRESSION
    if (SetBufferComPOCL) {
        int r = SetBufferComPOCL(InputBuffer(), CL_COMPRESSION_VBYTE, nullptr);
        assert (r == CL_SUCCESS);
        LOG_I ("Buffer compression VBYTE enabled\n");
    }
#endif

    GpuKernel.setArg(0, GpuInputBuffer);
    GpuKernel.setArg(1, InterBuffer);

    FpgaKernel.setArg(0, InterBuffer);
    FpgaKernel.setArg(1, FpgaOutputBuffer);

    initialized = true;
    return true;
}

bool OpenCL_Context::processCameraFrame(unsigned char* input, unsigned long *output) {
    if (!isAvailable()) {
        LOG_E("Device not available");
        return false;
    }

    std::unique_lock<std::mutex> lock(OclMutex);

#ifdef TIMING
    LOG_D("OpenCL: start processCameraFrame\n");
    auto start_time = std::chrono::steady_clock::now();
#endif

    cl_int err;
    *output = 0;

    std::vector<cl::Event> evts;
    cl::Event ev1, ev2, ev3, ev4;

    err = GpuQueue.enqueueWriteBuffer(GpuInputBuffer, CL_FALSE, 0, InputBufferSize, input, nullptr, &ev1);
    if (err != CL_SUCCESS)
        return false;

    evts.push_back(ev1);
    err = GpuQueue.enqueueNDRangeKernel(GpuKernel, Offset, Global, Local, &evts, &ev2);
    if (err != CL_SUCCESS)
        return false;

    err = FpgaQueue.enqueueWriteBuffer(FpgaOutputBuffer, CL_FALSE, 0, sizeof(cl_ulong), output, nullptr, &ev3);
    if (err != CL_SUCCESS)
        return false;

    evts.clear();
    evts.push_back(ev2);
    evts.push_back(ev3);
    err = FpgaQueue.enqueueNDRangeKernel(FpgaKernel, Offset, Global, Local, &evts, &ev4);
    if (err != CL_SUCCESS)
        return false;

    evts.clear();
    evts.push_back(ev4);
    err = GpuQueue.enqueueReadBuffer(FpgaOutputBuffer, CL_TRUE, 0, sizeof(unsigned long), output, &evts);
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

