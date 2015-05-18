cmake_minimum_required(VERSION 2.8.3)
project(scalebot)

find_package(catkin REQUIRED COMPONENTS
	  roscpp
	  rospy
	  std_msgs
	  sensor_msgs
)
catkin_package()

foreach(dir images launch urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(
  FILES urdf.rviz urdf.vcg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(joint_state_publisher src/track_angle_node.cpp)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})