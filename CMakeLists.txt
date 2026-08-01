cmake_minimum_required(VERSION 3.10)
project(BVINURingInterface VERSION 0.1.0 LANGUAGES C CXX)

# C++17 required for std::optional and structured bindings if used later
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Release build by default - enables compiler optimizations important for 90 Hz loop
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Write build/compile_commands.json on every configure. This is what VS Code's
# C/C++ IntelliSense reads (see .vscode/c_cpp_properties.json) so it knows where
# the OpenCV headers are (/usr/local/include/opencv4, not a default search path)
# and offers cv:: completions. Previously only set on the cmake command line, so
# a fresh configure elsewhere silently lost autocomplete.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Automatically picks up all .cpp files in src/ - add new handlers there.
# CONFIGURE_DEPENDS makes the build system re-glob on every build, so a newly
# added .cpp is picked up by a plain `ninja` with no manual `cmake .` step.
file(GLOB SRC_FILES CONFIGURE_DEPENDS "src/*.cpp")

add_executable(BVINURingInterface main.cpp ${SRC_FILES})

# ---- Link libraries ----------------------------------------------------------

target_link_libraries(BVINURingInterface PRIVATE
    ${OpenCV_LIBS}       # OpenCV (includes CUDA modules if OpenCV was built with CUDA)
    X11::X11             # X11 base
    ${XI_LIB}            # XInput2 (touchscreen)
    Threads::Threads     # pthreads (std::thread)
)

# DisplayHandler.cpp triggers a GCC 13 internal compiler error in the ud_dce
# RTL pass at -O3 due to the large PopulateControllerPanel() function. -O2
# avoids the ICE with no meaningful performance impact (this is GUI/drawing code).
set_source_files_properties(src/DisplayHandler.cpp PROPERTIES COMPILE_FLAGS "-O2")

# ---- Runtime resources -------------------------------------------------------

# Symlink config.yaml into the build directory so the binary finds it whether
# it is launched from the build directory or the project root.
# A symlink (not a copy) means edits to the source config.yaml take effect
# immediately - no rebuild needed.
add_custom_command(
    TARGET BVINURingInterface POST_BUILD
    COMMAND ln -sf ${CMAKE_SOURCE_DIR}/config.yaml ${CMAKE_BINARY_DIR}/config.yaml
    COMMENT "Symlinking config.yaml into build directory"
)
