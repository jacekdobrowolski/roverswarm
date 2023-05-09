cmake_minimum_required(VERSION 3.0.2)
project(roversim)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES
                  Color.msg Pose.msg)
add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  SetPen.srv
                  Spawn.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

set(roversim_node_SRCS
  src/roversim.cpp
  src/rover.cpp
  src/rover_frame.cpp
)
set(roversim_node_HDRS
  include/roversim/rover_frame.h
)

qt5_wrap_cpp(roversim_node_MOCS ${roversim_node_HDRS})


add_executable(roversim_node ${roversim_node_SRCS} ${roversim_node_MOCS})
target_link_libraries(roversim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(roversim_node roversim_gencpp)

add_executable(rover_teleop_key tutorials/teleop_rover_key.cpp)
target_link_libraries(rover_teleop_key ${catkin_LIBRARIES})
add_dependencies(rover_teleop_key roversim_gencpp)
add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square roversim_gencpp)
add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic roversim_gencpp)

install(TARGETS roversim_node rover_teleop_key draw_square mimic
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
