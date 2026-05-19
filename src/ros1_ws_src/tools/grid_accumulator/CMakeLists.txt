cmake_minimum_required(VERSION 3.0.2)
project(grid_accumulator)
add_compile_options(-std=c++17 -O3)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs tf2 tf2_ros tf2_sensor_msgs pcl_ros)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# HongTu-style ground cloud accumulator (body-frame ground extraction → global accumulation → PCD)
add_executable(ground_cloud_accumulator src/ground_cloud_accumulator.cpp)
target_link_libraries(ground_cloud_accumulator ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Original grid_accumulator (kept for reference, 2D occupancy grid approach)
add_executable(grid_accumulator_node src/grid_accumulator_node.cpp)
target_link_libraries(grid_accumulator_node ${catkin_LIBRARIES})

install(TARGETS ground_cloud_accumulator grid_accumulator_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
