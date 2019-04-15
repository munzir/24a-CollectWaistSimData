cmake_minimum_required(VERSION 3.0.0)

## set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")

project(waistKrang)

find_package(DART 6.7.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})

file(GLOB srcs "Main.cpp" "MyWindow.cpp" "MyWindow.hpp" "Controller.cpp" "Controller.hpp")
add_executable(${PROJECT_NAME} ${srcs})

target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} nlopt)
