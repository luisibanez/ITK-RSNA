cmake_minimum_required(VERSION 2.0)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



project(ConfidenceConnected)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

add_executable(ConfidenceConnected ConfidenceConnected.cxx)
target_link_libraries(ConfidenceConnected ${ITK_LIBRARIES})

