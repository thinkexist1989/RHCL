cmake_minimum_required(VERSION 3.1)

project(RHCL_test)

# test
add_executable(manual_registration manual_registration.cpp)
target_link_libraries(manual_registration RHCL::RHCL)

add_executable(test_yaml test_yaml.cpp)
#target_include_directories(test_yaml PUBLIC ${INCLUDES})
target_link_libraries(test_yaml RHCL::RHCL)

add_executable(test_kinect test_kinect.cpp)
#target_include_directories(test_kinect PUBLIC ${INCLUDES})
target_link_libraries(test_kinect RHCL::RHCL)

add_executable(test_ICamera test_ICamera.cpp)
#target_include_directories(test_ICamera PUBLIC ${CMAKE_BINARY_DIR})
target_link_libraries(test_ICamera RHCL::RHCL)