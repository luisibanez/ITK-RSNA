cmake_minimum_required(VERSION 2.0)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



project(MedianImageFilter)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

add_executable(MedianImageFilter MedianImageFilter.cxx)
target_link_libraries(MedianImageFilter ${ITK_LIBRARIES})

