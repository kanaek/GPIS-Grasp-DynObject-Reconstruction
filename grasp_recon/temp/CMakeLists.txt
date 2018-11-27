cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(octree_search)

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  std_msgs
  cv_bridge
  genmsg
  image_transport
  message_filters
  sensor_msgs
  geometry_msgs
  pcl_ros
  tf
  message_generation
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES using_markers
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable (octree_search octree_search.cpp)
target_link_libraries (octree_search ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable (octree_viewer octree_viewer.cpp)
#target_link_libraries (octree_search ${PCL_LIBRARIES})
