cmake_minimum_required(VERSION 2.8.3)
project(bb_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg tf)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")


# add_message_files(DIRECTORY msg )
# generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Generate messages in the 'msg' folder

catkin_package()


add_executable(himmelhilf src/hilfe.cpp)
target_link_libraries(himmelhilf ${catkin_LIBRARIES})
add_dependencies(himmelhilf bb_controller_gencpp )

add_executable(controlleraktuell src/tryeraktuell.cpp)
target_link_libraries(controlleraktuell ${catkin_LIBRARIES})
add_dependencies(controlleraktuell bb_controller_gencpp )

add_executable(ControllerToStartPID src/ControllerToStartPID.cpp)
target_link_libraries(ControllerToStartPID ${catkin_LIBRARIES})
add_dependencies(ControllerToStartPID bb_controller_gencpp )

add_executable(ControllerBase src/ControllerBase.cpp)
target_link_libraries(ControllerBase ${catkin_LIBRARIES})
add_dependencies(ControllerBase bb_controller_gencpp )

add_executable(Controller1PID src/Controller1PID.cpp)
target_link_libraries(Controller1PID ${catkin_LIBRARIES})
add_dependencies(Controller1PID bb_controller_gencpp )


add_executable(controllermickey src/controllermickey.cpp)
target_link_libraries(controllermickey ${catkin_LIBRARIES})
add_dependencies(controllermickey bb_controller_gencpp )

#add_executable(Visualizator src/Visualizator.cpp)
#target_link_libraries(Visualizator ${catkin_LIBRARIES})
#add_dependencies(Visualizator bb_controller_gencpp )



