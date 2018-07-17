cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

# Follow this format for adding new additions to the code using dependencies

add_executable(apriltags_demo apriltags_demo.cpp Serial.cpp)
pods_install_executables(apriltags_demo)

add_executable(imu imu.cpp Serial.cpp)
pods_install_executables(imu)
