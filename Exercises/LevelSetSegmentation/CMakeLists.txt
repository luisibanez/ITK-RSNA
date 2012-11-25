cmake_minimum_required(VERSION 2.0)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)



project(LevelSet)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(LevelSet LevelSet.cxx)
target_link_libraries(LevelSet ${ITK_LIBRARIES} ${VTK_LIBRARIES})

