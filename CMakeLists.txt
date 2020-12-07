cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

#if(POLICY CMP0074)
#    # 1. Remove with 3.12.4.
#    # 2. Remove search paths with *_ROOT since they will be automatically checked
#    cmake_policy(SET CMP0074 NEW)
#endif()
#
## see here: https://github.com/PointCloudLibrary/pcl/issues/3680
## when this is fixed, we can remove the following 3 lines.
#if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
#    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
#endif()

project(RHCL)

#find_package(PCL 1.2 CONFIG QUIET REQUIRED COMPONENTS common io registration visualization)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (manual_registration manual_registration.cpp)
target_link_libraries (manual_registration ${PCL_LIBRARIES})
