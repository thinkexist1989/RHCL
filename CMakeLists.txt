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

project(RHCL-PROJECT)

##
# TODO: RHCL
##
add_subdirectory(RHCL)

##
# TODO: RHCLVisualizer
##
#file(GLOB SRC "RHCLVisualizer/*.cpp" "RHCLVisualizer/*.h" "RHCLVisualizer/*.hpp" "RHCLVisualizer/*.ui" "RHCLVisualizer/*.qrc") #添加src目录下所有相关源文件
#
#add_executable(RHCLVisualizer ${SRC})
##target_include_directories(RHCLVisualizer PUBLIC ${VTK_INCLUDE_DIRS})
#target_link_libraries(RHCLVisualizer Qt5::Widgets Qt5::OpenGL Qt5::Core Qt5::Gui ${VTK_LIBRARIES})
add_subdirectory(RHCLVisualizer)

##
# TODO: Test
##
add_subdirectory(test)