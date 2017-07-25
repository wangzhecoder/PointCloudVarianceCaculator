cmake_minimum_required(VERSION 2.8.3)
project(varianceCalculator)
find_package(catkin REQUIRED
	COMPONENTS
		roscpp
		rospy
		pcl_ros
		sensor_msgs
)
generate_messages(DEPENDENCIES
	sensor_msgs
)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(varianceCalculator src/varianceCalculator.cpp)
target_link_libraries(varianceCalculator ${catkin_LIBRARIES})