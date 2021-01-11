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

# Enable C++11 standard
#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set ${freenect2_DIR} to let cmake find freenect2, then ${freenect2_INCLUDE_DIR} and ${freenect2_LIBRARIES}
if (UNIX AND NOT APPLE)
    set(freenect2_DIR $ENV{HOME}/freenect2/lib/cmake/freenect2)
elseif(WIN32)
    set(freenect2_DIR C:/dev/libfreenect2/lib/cmake/freenect2)
endif ()

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

# Qt5
find_package(Qt5 COMPONENTS Widgets Network Xml OpenGL REQUIRED)

# OpenGL
find_package(OpenGL REQUIRED)

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

include_directories(${PCL_INCLUDE_DIRS} ${ASSIMP_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${freenect2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${PCL_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(INCLUDES ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${freenect2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
#set(LIBS Qt5::Widgets Qt5::Network Qt5::Xml Qt5::OpenGL ${YAML_CPP_LIBRARIES} ${ASSIMP_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS})
set(LIBS Qt5::Widgets Qt5::Network Qt5::Xml Qt5::OpenGL ${ASSIMP_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${OpenCV_LIBS})


file(GLOB SRC "src/*.cpp" "include/*.hpp") #添加src目录下所有源文件

if (WIN32)
    set(LIBS ${LIBS} OpenGL::GL) #注意，windows下import target只有OpenGL::GL
elseif (UNIX AND NOT APPLE)
    set(LIBS ${LIBS} OpenGL::OpenGL m) #Linux下使用<math.h>需要libm.so链接
else ()
    set(LIBS)
endif (WIN32)

#link_directories(${ASSIMP_LIBRARY_DIRS})

# add shared library
add_library(RHCL SHARED ${SRC})
#target_include_directories(RHCL PUBLIC ${INCLUDES})
target_link_libraries(RHCL PUBLIC ${LIBS})
add_library(RHCL::RHCL ALIAS RHCL)



# test
add_executable(manual_registration test/manual_registration.cpp ${SRC})
target_link_libraries(manual_registration ${LIBS})

#add_executable(test test/test.cpp ${SRC})
#target_include_directories(test PUBLIC include)
#target_link_libraries(test ${LIBS})

add_executable(test_yaml test/test_yaml.cpp ${SRC})
#target_include_directories(test_yaml PUBLIC ${INCLUDES})
target_link_libraries(test_yaml ${LIBS})

add_executable(test_kinect test/test_kinect.cpp ${SRC})
#target_include_directories(test_kinect PUBLIC ${INCLUDES})
target_link_libraries(test_kinect ${LIBS})