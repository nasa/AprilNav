cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

# Follow this format for adding new additions to the code using dependencies

add_executable(AprilNav AprilNav.cpp Serial.cpp)
pods_install_executables(AprilNav)

add_executable(WaypointInput WaypointInput.cpp Serial.cpp)
pods_install_executables(WaypointInput)

add_executable(ATStripped ATStripped.cpp Serial.cpp)
pods_install_executables(ATStripped)
