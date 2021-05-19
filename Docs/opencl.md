# OpenCL camera sensor example

* __[Installation summary](#installation-summary)__
* __[Requirements](#requirements)__
* __[Code](#code)__

---
## Installation summary

<details>
   <summary>
	 You'll need access to https://github.com/cpc/pocl-remote.git
   </summary>

```sh

# TERMINAL 1

# checkout PoCL git
cd $HOME
git clone https://github.com/cpc/pocl-remote.git

# build the server
mkdir $HOME/pocl_server
cd $HOME/pocl_server
cmake -DCMAKE_BUILD_TYPE=Release $HOME/pocl-remote/server
make -j8

# run the server on 127.0.0.1:3000
./pocld -a 127.0.0.1 -p 3000

# TERMINAL 2

# go into carla git repository
cd /home/user/Carla

# run Setup.sh, this will build PoCL as static library
./Util/BuildTools/Setup.sh

# build libcarla
make LibCarla

# build the plugin
make CarlaUE4Editor

# run Carla, make sure PoCL server is running before launching it
make launch

# TERMINAL 3

cd /home/user/Carla

# run the camera example
cd PythonAPI/examples
python3 camera_opencl.py

```
</details>

---
## Requirements

* __PoCL server side.__ An OpenCL device visible to the pocld server is required. Can be checked with clinfo utility.

---
## Code

The OpenCL part lives in `LibCarla/source/carla/OpenCL/OpenCLcontext.cpp`,
as a simple class OpenCL_Context. This also contains the OpenCL kernel code.

The OpenCL camera sensor is in `LibCarla/source/carla/sensor` and
`Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/PixelReader.h`

It extends PixelReader with GetPixelsInRenderThread method,
which returns the captured frame in a `carla::Buffer`,
the sensor then calls an OpenCL_Context instance to process
the captured frame.
