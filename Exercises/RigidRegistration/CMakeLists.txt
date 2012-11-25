cmake_minimum_required(VERSION 2.0)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



project(RigidRegistration)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

add_executable(RigidRegistration RigidRegistration.cxx)
target_link_libraries(RigidRegistration ${ITK_LIBRARIES})

