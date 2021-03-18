cmake_minimum_required(VERSION 3.0.2)

project(flying_car_urdf)

find_package(catkin REQUIRED
	COMPONENTS
        roscpp
		rospy
		std_msgs
		nav_msgs
)
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		${PROJECT_NAME}
	CATKIN_DEPENDS
		roscpp
		rospy
		std_msgs
		nav_msgs
)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
# target_link_libraries(astar_node
    # ${Boost_LIBRARIES}
    # ${catkin_LIBRARIES}
# )
add_executable(astar_node 
	src/astar_node.cpp
)
target_link_libraries(astar_node
	${catkin_LIBRARIES}
)
add_dependencies(astar_node 
	${catkin_EXPORTED_TARGETS}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)