
---

## `CMakeLists.txt` (starter template)

This is a **best-effort** CMake scaffold for NAOqi + OpenCV. NAOqi SDK layouts vary, so you may need to tweak library names/paths.

```cmake
cmake_minimum_required(VERSION 3.10)
project(nao_vision_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- User config ----
# Pass with: -DAL_DIR=/path/to/naoqi-sdk
set(AL_DIR "" CACHE PATH "Path to NAOqi C++ SDK root")

if(NOT AL_DIR)
  message(FATAL_ERROR "Please set AL_DIR to your NAOqi SDK path (e.g. -DAL_DIR=/opt/naoqi-sdk).")
endif()

# ---- OpenCV ----
find_package(OpenCV REQUIRED)

# ---- NAOqi headers/libs ----
# Common SDK patterns:
#   ${AL_DIR}/include
#   ${AL_DIR}/lib
include_directories(${AL_DIR}/include)
link_directories(${AL_DIR}/lib)

add_executable(nao_vision_motion
  src/nao_main.cpp
)

add_executable(nao_vision_motion_alt_ip
  src/nao_main_alt_ip.cpp
)

# NAOqi libs commonly required by proxy-based C++ apps
# (names can differ depending on SDK version)
set(NAOQI_LIBS
  alcommon
  alproxies
  alvalue
  alerror
  alvision
  qi
)

target_include_directories(nao_vision_motion PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nao_vision_motion PRIVATE ${OpenCV_LIBS} ${NAOQI_LIBS})

target_include_directories(nao_vision_motion_alt_ip PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nao_vision_motion_alt_ip PRIVATE ${OpenCV_LIBS} ${NAOQI_LIBS})

