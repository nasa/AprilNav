cmake_minimum_required(VERSION 2.6)
project(apriltags)

#add_definitions(-pg) #"-fopenmp)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME apriltags)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
include(cmake/pods.cmake)

file(GLOB SOURCE_FILES "src/*.cc")
include_directories(AprilTags . /opt/local/include)
add_library(apriltags ${SOURCE_FILES})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltags ${OpenCV_LIBS} pthread) #-pg) #-fopenmp)
pods_use_pkg_config_packages(apriltags eigen3)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(apriltags -L/opt/local/lib/) # MacPorts special treatment...
else (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  pods_use_pkg_config_packages(apriltags libv4l2)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

pods_install_libraries(apriltags)

file(GLOB header_files "AprilTags/*.h")
pods_install_headers(${header_files} DESTINATION AprilTags/)

pods_install_pkg_config_file(apriltags
    LIBS -lapriltags
    REQUIRES eigen3 opencv2
    VERSION 1.0.0)

add_subdirectory(AprilNav)
add_subdirectory(example)

