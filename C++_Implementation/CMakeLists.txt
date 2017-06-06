cmake_minimum_required(VERSION 2.8)

project(EnKCF+ParticleFilter)

find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

# uncomment bellow if you want to use ground truth
# add_definitions( -D_USE_VIDEO_INPUT_ )
add_definitions( -D_UAV123_DATASET_)
# add_definitions( -D_AutelDATASET_)
# add_definitions( -D_VOT15_DATASET_ )
# add_definitions( -D_OTB100_DATASET_ )
FILE(GLOB_RECURSE sourcefiles ./main/src/runtracker.cpp ./detector/src/kcftracker.cpp ./detector/src/fhog.cpp ./tracker/src/Function_Definitions.cpp)

INCLUDE_DIRECTORIES(./detector/include/ ./tracker/include/)
add_executable( EnKCF ${sourcefiles} )
target_link_libraries( EnKCF ${OpenCV_LIBS})
