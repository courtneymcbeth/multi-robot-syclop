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


## kcbs

add_executable(mr_syclop
  src/mr_syclop.cpp
  src/decomposition.cpp
)
target_link_libraries(mr_syclop
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(mr_syclop PUBLIC
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)
