cmake_minimum_required(VERSION 3.16)
project(boids VERSION 0.1.0)

# enables support for testing, including the BUILD_TESTING option used below
include(CTest)

# Require the use of C++17, without non-standard extensions offered by the compiler 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# always compile with warnings
string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")

# enables address sanitizer in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address -fno-omit-frame-pointer")

# requires the graphic component in SFML library, version 2.5
#  dependencies are automatically identified
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

# adds executable boids, to build from the given source files 
# header files can, but don't have to, be reported as well
add_executable(boids main.cpp flock.cpp boids.cpp stats.cpp)
add_subdirectory(Lyra)
target_link_libraries(boids PRIVATE bfg::lyra)


# adds executable boid-sfml, to build from the given source files 
# header files can, but don't have to, be reported as well
add_executable(boids-sfml main-sfml.cpp boids.cpp flock.cpp stats.cpp)
# executable file depends on sfml-graphics and its dependencies
target_link_libraries(boids-sfml PRIVATE sfml-graphics)
target_link_libraries(boids-sfml PRIVATE bfg::lyra)
target_compile_definitions(boids-sfml PRIVATE GRAPHICS)


# if testing is enabled...
#  to disable testing, pass -DBUILD_TESTING=OFF to cmake during the configuration phase
if (BUILD_TESTING)

# add executables to build from the given source files 
 add_executable(parameters.t parameters.test.cpp)
 add_executable(boids.t boids.test.cpp boids.cpp)
 add_executable(flock.t flock.test.cpp flock.cpp boids.cpp)
 add_executable(stats.t stats.test.cpp stats.cpp flock.cpp boids.cpp)
# add executable parameters.t to test list, with the name parameters.t
  add_test(NAME parameters.t COMMAND parameters.t)
  add_test(NAME boids.t COMMAND boids.t)
  add_test(NAME flock.t COMMAND flock.t)
  add_test(NAME stats.t COMMAND stats.t)

 


endif()
