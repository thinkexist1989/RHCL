cmake_minimum_required(VERSION 3.10)

project(RHCLVisualizer)

# Enable C++11 standard
set(CMAKE_CXX_STANDARD 11)

# RHCL
#find_package(RHCL REQUIRED)

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE}) #必须加这句话，否则报错

#Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

file(GLOB SRC "*.cpp" "*.h" "*.hpp" "*.ui" "*.qrc") #添加src目录下所有相关源文件

add_executable(RHCLVisualizer ${SRC})
target_include_directories(RHCLVisualizer PUBLIC ${VTK_INCLUDE_DIRS})
target_link_libraries(RHCLVisualizer RHCL::RHCL Qt5::Widgets ${VTK_LIBRARIES})