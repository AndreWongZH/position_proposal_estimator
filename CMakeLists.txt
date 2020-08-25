cmake_minimum_required(VERSION 3.5)
project(position_proposal_estimator)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-pg")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

find_package(Pangolin REQUIRED)

find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
find_package(TBB REQUIRED)


add_subdirectory(thirdparty/yaml-cpp)

find_package(yaml-cpp REQUIRED)

include_directories(
    include
    thirdparty/basalt-headers/include
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIR}
)

add_library(multiview_visualiser
    src/multiview_visualiser.cpp
)

#add_library(depth_verification
#    src/depth_verification.hpp
#)

target_link_libraries(multiview_visualiser
PRIVATE
    ${Pangolin_LIBRARIES}
PUBLIC
    ${TBB_LIBRARIES}
)

add_executable(multiview_geometry_test
    src/multiview_geometry_test.cpp
)
add_executable(depth_verification_test
    src/depth_verification_test.cpp
)

# target_include_directories(multiview_geometry_test
# PRIVATE
#     # ${YAML_CPP_INCLUDE_DIR}
# )

target_link_libraries(multiview_geometry_test
PRIVATE
    # ${YAML_CPP_LIBRARIES}
    yaml-cpp
    multiview_visualiser
    ${OpenCV_LIBS}
)

target_link_libraries(depth_verification_test
    yaml-cpp
    multiview_visualiser
    ${OpenCV_LIBS}
)
