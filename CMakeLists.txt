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
#find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

# adds executable boid, to build from the given source files 
# header files can, but don't have to, be reported as well
add_executable(boids main.cpp)
add_subdirectory(Lyra)
target_link_libraries(boids PUBLIC bfg::lyra)


# adds executable boid-sfml, to build from the given source files 
# header files can, but don't have to, be reported as well
# add_executable(boids-sfml main-sfml.cpp boid.cpp flock.cpp parameters.cpp)
# executable file depends on sfml-graphics and its dependencies
# target_link_libraries(boids-sfml PRIVATE sfml-graphics)

# if testing is enabled...
#  to disable testing, pass -DBUILD_TESTING=OFF to cmake during the configuration phase
#if (BUILD_TESTING)

  #add executable chain.t, to build from the given source files 
 # add_executable(boid.t boid.test.cpp boid.cpp)
 # target_link_libraries(boid.t PRIVATE sfml-graphics)
  # add executable chain.t to test list, with the name chain.t
  #add_test(NAME boid.t COMMAND boid.t)

#endif()
