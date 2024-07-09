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

#include "OpenCLcontext2.h"

#ifdef LIBCARLA_INCLUDED_FROM_UE4
#include "carla/Debug.h"
#include "carla/Logging.h"
#endif

/* this is rather silly, Carla disables exceptions so we have to undef
 * CL_HPP_ENABLE_EXCEPTIONS, except there are still codepaths that throw
 * exceptions if the OpenCL version >= 200. Argh. */
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_TARGET_OPENCL_VERSION 120
#undef CL_HPP_ENABLE_EXCEPTIONS

/* AlmaIF apparently only supports 1.2, so request that it be used when building
 * the program. */
#define CL_HPP_CL_1_2_DEFAULT_BUILD
#include <CL/cl2.hpp>
#include <assert.h>

#include <iostream>

#define LIDARDATADEF() \
  typedef struct { \
    float x; \
    float y; \
    float z; \
    float intensity; \
  } LidarData;

#define POINTDEF() \
  typedef struct { \
    int x; \
    int y; \
  } Point;

LIDARDATADEF();
POINTDEF();

#define QUOTE2(x) #x
#define QUOTE(x) QUOTE2(x)

const std::string GRIDMAP_SOURCE =
QUOTE(LIDARDATADEF())
QUOTE(POINTDEF())
  R"(
#define OCC -1
#define FREE 10
#define PROB_FREE log(0.35f/0.45f)

#define map_at(map, xw, yw, i, j)\
        map[i * xw + j]

float occupied_with_intensity(float intensity) {
    return log(intensity / (1.0f - intensity));
}

int bresenham(Point start, Point end, __private Point points[64], int cloud_size) {
    // Setup initial conditions
    int x1 = start.x;
    int y1 = start.y;
    int x2 = end.x;
    int y2 = end.y;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int is_steep = abs(dy) > abs(dx);

    if (is_steep) {
        // Rotate line
        int temp = x1;
        x1 = y1;
        y1 = temp;
        temp = x2;
        x2 = y2;
        y2 = temp;
    }

    // Swap start and end points if necessary and store swap state
    int swapped = 0;
    if (x1 > x2) {
        int temp = x1;
        x1 = x2;
        x2 = temp;
        temp = y1;
        y1 = y2;
        y2 = temp;
        swapped = 1;
    }

    dx = x2 - x1;
    dy = y2 - y1;
    int error = dx / 2;
    int y_step = (y1 < y2) ? 1 : -1;

    // Iterate over the bounding box generating points between start and end
    int y = y1;
    int num = 0;
    for (int x = x1; x <= x2; x++) {
        Point coord;
        coord.x = is_steep ? y : x;
        coord.y = is_steep ? x : y;

        // Append the point to the array
        points[num] = coord;
        num++;

        error -= abs(dy);
        if (error < 0) {
            y += y_step;
            error += dx;
        }
    }

    if (swapped) {
        // Reverse the list if the coordinates were swapped
        int i, j;
  for (i = 0, j = num - 1; i < j; i++, j--) {
    Point temp = points[i];
    points[i] = points[j];
    points[j] = temp;
  }
}

return num;
}

__kernel void
generate_gridmap(__global LidarData *arr,
    int cloud_size,
    int xw, int yw, int center_x, int center_y, int min_x, int min_y,
    __global float *occupancy_map, float xy_resolution)
{
  size_t k = get_global_id(0);
  if (isnan(arr[k].x) || isnan(arr[k].y)) {
    // Handle NaN values (you can skip or log these points)
    //printf("found NaN, aborting\n");
    return;
  }

  int ix = abs((int)round((arr[k].x - min_x) / xy_resolution));
  int iy = abs((int)round((arr[k].y - min_y) / xy_resolution));

  if (!((abs(ix) < xw) && abs(iy) < yw))
    return;

  Point LidarPoint;
  Point Lidar;
  Point points[64]; // is this enough?
  LidarPoint.x = ix;
  LidarPoint.y = iy;
  Lidar.x = center_x;
  Lidar.y = center_y;

  int num_of_points = bresenham(Lidar, LidarPoint, points, cloud_size);
  if (num_of_points >= 64) {
    //printf("number of points larger than point buffer, aborting\n");
    return;
  }

  for (int i = 0; i < num_of_points; i++) {
    // freespace between
    map_at(occupancy_map, xw, yw, points[i].x, points[i].y) -= PROB_FREE;
    if (map_at(occupancy_map, xw, yw, points[i].x, points[i].y) > FREE)
      map_at(occupancy_map, xw, yw, points[i].x, points[i].y) = FREE;
  }

  float occ = occupied_with_intensity(arr[k].intensity);
  map_at(occupancy_map, xw, yw, ix, iy) -= occ; // Occupied area

  if (map_at(occupancy_map, xw, yw, ix, iy) < OCC)
    map_at(occupancy_map, xw, yw, ix, iy) = OCC;
}
)";

class OpenCL_Context2 {
  public:
    bool init();
    bool genGridMap(LidarData *arr,
        int cloud_size, int xw, int yw,
        int center_x, int center_y, int min_x, int min_y,
        float *occupancy_map, float xy_resolution);

  private:
    cl::Context context;
    cl::Platform platform;
    cl::CommandQueue queue;
    cl::Program program;
    cl::Kernel kernel;

    std::vector<cl::Device> devices;
    cl::Device device;

    bool ok = false;
};

#define CHECK_CL_ERROR(e, ...)\
  if ((e) != CL_SUCCESS) { carla::log_error(__VA_ARGS__); return false; }

bool OpenCL_Context2::init()
{
  cl_int err;

  /* just get first platform + device for now */
  err = cl::Platform::get(&platform);
  CHECK_CL_ERROR(err, "Failed getting OpenCL platform\n");

  /* we really only use the first one but heyho */
  err = platform.getDevices(CL_DEVICE_TYPE_ALL, &devices);
  CHECK_CL_ERROR(err, "Failed getting platform devices\n");

  if (devices.size() == 0) {
    carla::log_error("No OpenCL devices available\n");
    return false;
  }

  device = devices[0];

  context = cl::Context(devices, nullptr, nullptr, nullptr, &err);
  CHECK_CL_ERROR(err, "Failed creating an OpenCL context\n");

  queue = cl::CommandQueue(context, device, 0, &err);
  CHECK_CL_ERROR(err, "Failed creating an OpenCL command queue\n");

  program = cl::Program(context, GRIDMAP_SOURCE, false, &err);
  CHECK_CL_ERROR(err, "Failed creating program\n");

  /* Unsure if this is a bug in the compiler or what but the kernel will do some
   * out-of-bounds accesses with optimizations enabled */
  err = program.build("-cl-opt-disable");
  if (err != CL_SUCCESS) {
    carla::log_error("Failed building OpenCL program\n");
    auto info = program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(&err);
    for (auto &pair : info)
      std::cerr << pair.second << std::endl;

    return false;
  }

  kernel = cl::Kernel(program, "generate_gridmap", &err);
  CHECK_CL_ERROR(err, "Failed creating a kernel\n");

  return ok = true;
}

/* base case */
template <size_t>
cl_int setArgs(cl::Kernel) { return CL_SUCCESS; }

/* recursive case */
template<size_t index = 0, typename T, typename ...Ts>
cl_int setArgs(cl::Kernel kernel, T&& t, Ts&& ...ts)
{
  cl_int err = kernel.setArg(index, t);
  if (err != CL_SUCCESS)
    return err;

  return setArgs<index + 1, Ts...>(kernel, std::forward<Ts>(ts)...);
}

bool OpenCL_Context2::genGridMap(LidarData *arr,
    int cloud_size, int xw, int yw,
    int center_x, int center_y, int min_x, int min_y,
    float *occupancy_map, float xy_resolution)
{
  if (!ok)
    return false;

  cl_int err;
  auto input = cl::Buffer(context,
      CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
      cloud_size * sizeof(LidarData), arr, &err);
  CHECK_CL_ERROR(err, "Failed creating input buffer\n");

  auto output = cl::Buffer(context,
      CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
      xw * yw * sizeof(float), occupancy_map, &err);
  CHECK_CL_ERROR(err, "Failed creating output buffer\n");

  err = setArgs(kernel, input, cloud_size,
      xw, yw, center_x, center_y, min_x, min_y,
      output, xy_resolution);

  CHECK_CL_ERROR(err, "Failed setting kernel args\n");

  err = queue.enqueueNDRangeKernel(kernel, cl::NDRange(0), cl::NDRange(cloud_size));
  CHECK_CL_ERROR(err, "Failed enqueueing kernel\n");

  err = queue.enqueueReadBuffer(output, CL_TRUE,
      0, xw * yw * sizeof(cl_float), occupancy_map);
  CHECK_CL_ERROR(err, "Failed reading output buffer\n");

  return true;
}

OpenCL_Manager2::OpenCL_Manager2()
{
  Context = std::unique_ptr<OpenCL_Context2>{new OpenCL_Context2{}};
  Context->init();
}

OpenCL_Manager2::~OpenCL_Manager2()
{
  Context.release();
}

void OpenCL_Manager2::GenerateGridmap(std::vector<float> Data, carla::Buffer& Buffer)
{
  float xy_resolution = 0.5;
  float min_x = -25.0;
  float min_y = -25.0;
  float max_x = 25.0;
  float max_y = 25.0;

  int xw = (int)round((max_x - min_x) / xy_resolution);
  int yw = (int)round((max_y - min_y) / xy_resolution);

  int center_x = (int)round(xw * xy_resolution);
  int center_y = (int)round(yw * xy_resolution);

  size_t cloud_size = Data.size();
  LidarData *filtered_data = (LidarData *)malloc(cloud_size * sizeof(float));

  size_t point_count = 0; size_t size = 0;
  while (point_count < cloud_size) {
    LidarData lidar_point = {};
    lidar_point.x = Data[point_count++];
    lidar_point.y = Data[point_count++];
    lidar_point.z = Data[point_count++];
    lidar_point.intensity = Data[point_count];

    float dist = sqrtf(powf(lidar_point.x, 2) + powf(lidar_point.y, 2));
    if (lidar_point.z > -1.8 && dist < 25 && lidar_point.x != 0 && lidar_point.y != 0) {
      filtered_data[size] = lidar_point;
      size++;
    }

    point_count++;
  }

  float *occupancy_map = (float *)malloc(xw * yw * sizeof(float));
  for (size_t i = 0; i < xw * yw; ++i) {
    occupancy_map[i] = log(0.5/0.5);
  }


  Context->genGridMap(filtered_data, size,
      xw, yw, center_x, center_y, min_x, min_y,
      occupancy_map, 0.5);

  Buffer.copy_from((const carla::Buffer::value_type *)occupancy_map, xw * yw * sizeof(float));
  free(occupancy_map);
  free(filtered_data);
}
