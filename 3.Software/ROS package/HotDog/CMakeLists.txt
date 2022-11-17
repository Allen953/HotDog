cmake_minimum_required(VERSION 2.8.3)

project(HotDog)

find_package(catkin REQUIRED)

find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  controller_manager
  gazebo_ros
  gazebo_ros_control
  joint_state_publisher
  robot_state_publisher
  roscpp
  rospy
  rviz
  xacro
  kdl_parser
  tf
  sensor_msgs
  std_msgs
  trac_ik_lib
  serial
  robot_msg
)


# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)
#执行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(baizeOne_flucturate src/baizeOne_flucturate.cpp)
target_link_libraries(baizeOne_flucturate ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
