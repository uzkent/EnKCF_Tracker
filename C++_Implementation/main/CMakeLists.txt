cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )

# add_definitions(-D_USE_ROUTER_)
SET(USERNAME  "/home/$ENV{USER}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
SET(SRC_LISTS ./src/main.cpp ./src/interface.cpp ../edge/src/edge_boxes_interface.cpp ../edge/src/edge_box.cpp ../edge/src/edge_detection.cpp ../tools/src/conv_triangle.cpp ../tools/src/gradient.cpp ../tools/src/image_tools.cpp ../tools/src/data_define.cpp)
INCLUDE_DIRECTORIES(./main/include/ ../edge/include/ ../tools/include/)
ADD_EXECUTABLE(edgebox ${SRC_LISTS})
target_link_libraries(edgebox ${OpenCV_LIBS})
