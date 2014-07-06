cmake_minimum_required(VERSION 2.8.3)
project(bb_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg tf )
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
MESSAGE(${EIGEN3_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


# add_message_files(DIRECTORY msg )
# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Generate messages in the 'msg' folder

catkin_package()


add_executable(controlleraktuell src/tryeraktuell.cpp)
target_link_libraries(controlleraktuell ${catkin_LIBRARIES})
add_dependencies(controlleraktuell bb_controller_gencpp )

add_executable(Controller src/Controller.cpp)
target_link_libraries(Controller ${catkin_LIBRARIES})
target_link_libraries(Controller ${Eigen3_LIBRARIES})
add_dependencies(Controller bb_controller_gencpp )

add_executable(ControllerToStartPID src/ControllerToStartPID.cpp)
target_link_libraries(ControllerToStartPID ${catkin_LIBRARIES})
add_dependencies(ControllerToStartPID bb_controller_gencpp )
