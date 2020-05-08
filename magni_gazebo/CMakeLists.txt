cmake_minimum_required(VERSION 2.8.3)
project(magni_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})





catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 

)



