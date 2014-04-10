cmake_minimum_required(VERSION 2.8.3)
project(bb_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg tf)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


# add_message_files(DIRECTORY msg )
# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Generate messages in the 'msg' folder

catkin_package()

add_executable(bb_controller_feedfwd src/Controller.cpp)
target_link_libraries(bb_controller_feedfwd ${catkin_LIBRARIES})
add_dependencies(bb_controller_feedfwd bb_controller_gencpp )

add_executable(Controller10.4. src/Controller10.4..cpp)
target_link_libraries(Controller10.4. ${catkin_LIBRARIES})
add_dependencies(Controller10.4. bb_controller_gencpp )
# add_executable(io_node src/IONode.cpp)
# target_link_libraries(io_node ${catkin_LIBRARIES})
# add_dependencies(io_node bb_state_gencpp)

