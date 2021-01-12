cmake_minimum_required(VERSION 3.10)

if (POLICY CMP0074)
    # 1. Remove with 3.12.4.
    # 2. Remove search paths with *_ROOT since they will be automatically checked
    cmake_policy(SET CMP0074 NEW)
endif ()
#
## see here: https://github.com/PointCloudLibrary/pcl/issues/3680
## when this is fixed, we can remove the following 3 lines.
if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

project(RHCL)

# Enable C++11 standard
set(CMAKE_CXX_STANDARD 11)

# set ${freenect2_DIR} to let cmake find freenect2, then ${freenect2_INCLUDE_DIR} and ${freenect2_LIBRARIES}
if (UNIX AND NOT APPLE)
    set(freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)
elseif(WIN32)
    set(freenect2_DIR C:/dev/libfreenect2/lib/cmake/freenect2)
endif ()

##Qt5
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui OpenGL)

#Boost
find_package(Boost REQUIRED)
message("Boost DIR: ${Boost_INCLUDE_DIRS}")

# Assimp
find_package(assimp REQUIRED)
message("ASSIMP DIR: ${ASSIMP_INCLUDE_DIRS}")
message("ASSIMP LIBS DIR: ${ASSIMP_LIBRARY_DIRS}")
message("ASSIMP LIBS: ${ASSIMP_LIBRARIES}")

# PCL
#find_package(PCL 1.2 CONFIG QUIET REQUIRED COMPONENTS common io registration visualization)
find_package(PCL 1.2 REQUIRED)

## VTK
#find_package(VTK REQUIRED)
##include(${VTK_USE_FILE})

# YAML-CPP
find_package(yaml-cpp CONFIG REQUIRED)
message("yaml-cpp DIR: ${YAML_CPP_INCLUDE_DIR}")
message("yaml-cpp LIBS DIR: ${YAML_CPP_LIBRARY_DIR}")
message("yaml-cpp LIBS: ${YAML_CPP_LIBRARIES}")

# freenect2
find_package(freenect2 REQUIRED)
message("freenect2 DIR: ${freenect2_INCLUDE_DIRS}")
message("freenect2 LIBS: ${freenect2_LIBRARIES}")

# OpenCV
find_package(OpenCV REQUIRED)

#include_directories(${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${freenect2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(INCLUDES ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${freenect2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
#set(LIBS Qt5::Widgets Qt5::Network Qt5::Xml Qt5::OpenGL ${YAML_CPP_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS})
set(LIBS ${ASSIMP_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS})


file(GLOB SRC "*.cpp" "*.c") #添加src目录下所有源文件

#link_directories(${ASSIMP_LIBRARY_DIRS})

# add shared library
add_library(RHCL STATIC ${SRC})
add_library(RHCL::RHCL ALIAS RHCL)
target_include_directories(RHCL PUBLIC ${VTK_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${freenect2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_directories(RHCL PUBLIC ${ASSIMP_LIBRARY_DIRS})
target_link_libraries(RHCL PUBLIC ${ASSIMP_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS})
