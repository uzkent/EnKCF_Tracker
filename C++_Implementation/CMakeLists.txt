cmake_minimum_required(VERSION 2.8)
project(KCF+ParticleFilter)

find_package(OpenCV REQUIRED)

if(NOT WIN32)
   set(CMAKE_CXX_FLAGS "-std=c++11")
endif(NOT WIN32)

# uncomment bellow if you want to use ground truth
# add_definitions( -D_USE_GROUND_TRUTH_ )
include_directories(src) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")
add_executable( KCF ${sourcefiles} )
target_link_libraries( KCF ${OpenCV_LIBS})

