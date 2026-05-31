cmake_minimum_required(VERSION 3.5.0)
project(NURingIntegratedController VERSION 0.1.0 LANGUAGES C CXX)

# Set the C++ standard to use (for example, C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find necessary pacakges 
find_package(OpenCV REQUIRED)	
find_package(X11 REQUIRED)

# Add this if not already present
find_library(XI_LIB Xi)

# Add include directories
include_directories(${OpenCV_INCLUDE_DIRS})	
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create a pointer to the source files
file(GLOB SRC_FILES "src/*.cpp" ) 
add_executable(NURingIntegratedController main.cpp ${SRC_FILES})


# Link libraries
target_link_libraries(NURingIntegratedController PRIVATE X11::X11 Xi ${OpenCV_LIBS})

