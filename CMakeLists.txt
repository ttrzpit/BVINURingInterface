cmake_minimum_required(VERSION 3.10)
project(BVINURingInterface VERSION 0.1.0 LANGUAGES C CXX)

# C++17 required for std::optional and structured bindings if used later
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Release build by default — enables compiler optimizations important for 90 Hz loop
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---- Find dependencies ------------------------------------------------------

find_package(OpenCV REQUIRED)       # Must be built with CUDA support for GPU pipeline
find_package(X11 REQUIRED)          # X11 base library (window system)
find_package(Threads REQUIRED)      # std::thread support
find_library(XI_LIB Xi REQUIRED)   # XInput2 for touchscreen events

# ---- Include directories -----------------------------------------------------

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include      # Our own headers (Config.h, CameraHandler.h, etc.)
)

# ---- Source files ------------------------------------------------------------

# Automatically picks up all .cpp files in src/ — add new handlers there
file(GLOB SRC_FILES "src/*.cpp")

add_executable(BVINURingInterface main.cpp ${SRC_FILES})

# ---- Link libraries ----------------------------------------------------------

target_link_libraries(BVINURingInterface PRIVATE
    ${OpenCV_LIBS}       # OpenCV (includes CUDA modules if OpenCV was built with CUDA)
    X11::X11             # X11 base
    ${XI_LIB}            # XInput2 (touchscreen)
    Threads::Threads     # pthreads (std::thread)
)
