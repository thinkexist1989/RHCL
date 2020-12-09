cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# PCL
find_package(PCL 1.2 CONFIG QUIET REQUIRED COMPONENTS common io registration visualization)
# find_package(PCL 1.2 REQUIRED)

# Qt5
find_package(Qt5 COMPONENTS Widgets Network Xml OpenGL REQUIRED)

# Assimp
find_package(Assimp REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBS Qt5::Widgets Qt5::Network Qt5::Xml Qt5::OpenGL ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES})

file(GLOB SRC "src/*.cpp") #添加src目录下所有源文件

if (WIN32)
    set(LIBS ${LIBS} OpenGL::GL) #注意，windows下import target只有OpenGL::GL
elseif (UNIX AND NOT APPLE)
    set(LIBS ${LIBS} OpenGL::OpenGL m) #Linux下使用<math.h>需要libm.so链接
else ()
    set(LIBS)
endif (WIN32)

link_directories(${ASSIMP_LIBRARY_DIRS})


# test
add_executable(manual_registration test/manual_registration.cpp ${SRC})
target_link_libraries(manual_registration ${LIBS})

add_executable(test test/test.cpp ${SRC})
target_include_directories(test PUBLIC include)
target_link_libraries(test ${LIBS})