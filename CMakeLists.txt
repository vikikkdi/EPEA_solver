cmake_minimum_required(VERSION 3.10)
project(EPEA_solver)
set (CMAKE_CXX_STANDARD 17)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
#add_subdirectory(library/yaml-cpp)

add_library(epea INTERFACE)
target_include_directories(
  epea
  INTERFACE 
  include 
  library/yaml-cpp/include
)

add_executable(
  epea_mapf
  example/epea_test.cpp
)
target_link_libraries(
  epea_mapf
  epea
  yaml-cpp
  ${Boost_LIBRARIES}
)
