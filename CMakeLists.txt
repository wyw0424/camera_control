cmake_minimum_required(VERSION 2.8.3)
project(camera_control)

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS 
			roscpp 
			std_msgs
            geometry_msgs
			tf
)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(camera_control_node src/camera_control_node.cpp)
target_link_libraries(camera_control_node ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")