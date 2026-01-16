cmake_minimum_required(VERSION 3.5)

project(mr-syclop)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
find_package(fcl REQUIRED)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_subdirectory(Multi-Robot-OMPL EXCLUDE_FROM_ALL)

# Force db-CBS/dynoplan to use the same OMPL as Multi-Robot-OMPL
# This prevents linking conflicts between OMPL v17 (local) and v18 (system)
set(OMPL_FOUND TRUE CACHE BOOL "OMPL found" FORCE)
set(OMPL_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/Multi-Robot-OMPL/src;${CMAKE_BINARY_DIR}/Multi-Robot-OMPL/src" CACHE STRING "OMPL include directories" FORCE)
set(OMPL_LIBRARIES ompl CACHE STRING "OMPL libraries" FORCE)

# eigenpy, pinocchio, and crocoddyl are built separately by build.sh and installed to ./install
add_subdirectory(db-CBS EXCLUDE_FROM_ALL)

include_directories(
  ${FCL_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/Multi-Robot-OMPL/src
  ${CMAKE_BINARY_DIR}/Multi-Robot-OMPL/src
  ${CMAKE_SOURCE_DIR}/db-CBS/src
  ${CMAKE_SOURCE_DIR}/db-CBS/dynoplan/include
  ${CMAKE_SOURCE_DIR}/db-CBS/dynoplan/dynobench/include
  ${CMAKE_SOURCE_DIR}/install/include
)
# eigenpy, pinocchio, and crocoddyl are installed to ./install and included above

# add_library(motion_planning_common
#   src/robots.cpp
# )
# target_link_libraries(motion_planning_common
#   PUBLIC ompl
#   PUBLIC Eigen3::Eigen
#   PUBLIC ${FCL_LIBRARIES}
# )
# target_include_directories(motion_planning_common
#   PRIVATE ${CMAKE_BINARY_DIR}/deps/fcl/include
#   PRIVATE ${CMAKE_SOURCE_DIR}/deps/fcl/include
# )
# set_property(TARGET motion_planning_common PROPERTY POSITION_INDEPENDENT_CODE ON)


## mapf_solver library

add_library(mapf_solver_lib
  src/mapf/mapf_solver.cpp
  src/mapf/mapf_decoupled.cpp
  src/mapf/mapf_cbs.cpp
)
target_link_libraries(mapf_solver_lib
  PUBLIC ompl
  PUBLIC ${Boost_LIBRARIES}
)
target_include_directories(mapf_solver_lib PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/Multi-Robot-OMPL/src
  ${CMAKE_BINARY_DIR}/Multi-Robot-OMPL/src
)
# target_compile_options(mapf_solver_lib PRIVATE -g -O0)


## guided_planner library

add_library(guided_planner
  src/guided/guided_planner.cpp
  src/guided/syclop_rrt_solver.cpp
  src/guided/db_rrt_solver.cpp
)
target_link_libraries(guided_planner
  PUBLIC ompl
  PUBLIC ${FCL_LIBRARIES}
  PUBLIC motion_planning_common
  PUBLIC idbastar::dbrrt
  PUBLIC idbastar::optimization
  PUBLIC dynobench::dynobench
)
target_include_directories(guided_planner PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/guided
  ${CMAKE_SOURCE_DIR}/Multi-Robot-OMPL/src
  ${CMAKE_BINARY_DIR}/Multi-Robot-OMPL/src
  ${CMAKE_SOURCE_DIR}/db-CBS/dynoplan/include
  ${CMAKE_SOURCE_DIR}/db-CBS/dynoplan/dynobench/include
  ${FCL_INCLUDE_DIRS}
)
# Force debug flags for guided_planner regardless of build type
target_compile_options(guided_planner PRIVATE -g -O0)

## mr_syclop

add_executable(mr_syclop
  src/mr_syclop.cpp
  src/decomposition.cpp
)
target_link_libraries(mr_syclop
  motion_planning_common
  coupled_rrt_lib
  mapf_solver_lib
  guided_planner
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(mr_syclop PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)
# Force debug flags for mr_syclop regardless of build type
target_compile_options(mr_syclop PRIVATE -g -O0)

## srrt

add_executable(srrt
  src/srrt.cpp
)
target_link_libraries(srrt
  motion_planning_common
  ompl
  ${FCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
)
target_include_directories(srrt PUBLIC
  ${FCL_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

## coupled_rrt library and executable

# Create library for coupled RRT implementation (can be used by other targets)
add_library(coupled_rrt_lib
  src/coupled_rrt.cpp
)
target_link_libraries(coupled_rrt_lib
  PUBLIC motion_planning_common
  PUBLIC ompl
  PUBLIC ${FCL_LIBRARIES}
)
target_include_directories(coupled_rrt_lib PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${FCL_INCLUDE_DIRS}
)

# Executable that uses the library
add_executable(coupled_rrt
  src/coupled_rrt_main.cpp
)
target_link_libraries(coupled_rrt
  coupled_rrt_lib
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
)
target_include_directories(coupled_rrt PUBLIC
  ${YAML_INCLUDE_DIRS}
)

## decoupled_rrt

add_executable(decoupled_rrt
  src/decoupled_rrt.cpp
)
target_link_libraries(decoupled_rrt
  motion_planning_common
  ompl
  ${FCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
)
target_include_directories(decoupled_rrt PUBLIC
  ${FCL_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

## drrt

add_executable(drrt
  src/drrt.cpp
)
target_link_libraries(drrt
  motion_planning_common
  ompl
  ${FCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
)
target_include_directories(drrt PUBLIC
  ${FCL_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)

## arc (Adaptive Robot Coordination) library and executable

# Create library for ARC implementation (can be used by other targets)
add_library(arc_lib
  src/arc.cpp
)
target_link_libraries(arc_lib
  PUBLIC motion_planning_common
  PUBLIC ompl
  PUBLIC ${FCL_LIBRARIES}
)
target_include_directories(arc_lib PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${FCL_INCLUDE_DIRS}
)

# Executable that uses the library
add_executable(arc
  src/arc_main.cpp
)
target_link_libraries(arc
  arc_lib
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
)
target_include_directories(arc PUBLIC
  ${YAML_INCLUDE_DIRS}
)
