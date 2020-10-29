# run this,  on linux, as:
#make a directory build/ with `mkdir build`
#mkdir build/
#cd build
#cmake ../
#cmake --build .


cmake_minimum_required(VERSION 3.10)

# set the project name
project(Heuristic VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
#To enable compiler optimisation either for GCC or MSVS
#-DDEBUG stands for -D<macroname>=<value> which defines a macro
# -g: inlcude debug information when compiling. -O3 maximum optimisation level. -Wall -Wpedantic -Wextra: print/enable warnings. 
# -DDEBUG (is a composition of -D DEBUG): to define a macro named DEBUG (to be used later with `ifdef DEBUG` do something)
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # using Clang 
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O3 -Wall -Wpedantic -Wextra -Wno-sign-compare -DDEBUG")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # using GCC
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O3 -Wall -Wpedantic -Wextra -Wno-sign-compare -DDEBUG")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # using Visual Studio C++
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /O2 /Wall /Zi /DDEBUG")
endif()

# add the executable
add_executable(Heuristic Main.cpp Heuristic.cpp ProbabilityEstimator.cpp)

# In case installing as executable is needed
#Uncomment the following line, then after building do  "cd build && make install"
# install(TARGETS Heuristic DESTINATION bin)
