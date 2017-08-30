cmake_minimum_required(VERSION 2.8.3)

project(intensity_map_builder)

set (CMAKE_CXX_STANDARD 11)
find_package(Boost COMPONENTS boost_random property_tree)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
		cmake_modules 
		tf 
		message_generation
		geometry_msgs 
		std_msgs
		intensity_map_msgs
	    )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
catkin_package(
		CATKIN_DEPENDS intensity_map_msgs
		DEPENDS Boost
		INCLUDE_DIRS include
		LIBRARIES intensity_map_lib intensity_map_builder)

add_library(intensity_map_lib SHARED src/intensity_map_lib.cpp)
add_library(intensity_map_builder SHARED src/intensity_map_builder.cpp)

target_link_libraries(intensity_map_lib ${catkin_LIBRARIES} ${BOOST_LIBRARIES} )
target_link_libraries(intensity_map_builder ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

add_executable(intensity_map_builder_node src/intensity_map_builder_node.cpp)
target_link_libraries(intensity_map_builder_node ${catkin_LIBRARIES} ${BOOST_LIBRARIES} intensity_map_lib intensity_map_builder)


install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/intensity_map_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

#catkin_install_python(PROGRAMS scripts/show_map.py
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY include/intensity_map_lib/ include/intensity_map_builder/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS intensity_map_builder_node intensity_map_lib intensity_map_builder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


