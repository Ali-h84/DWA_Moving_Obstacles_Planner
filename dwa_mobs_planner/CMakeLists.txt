cmake_minimum_required(VERSION 2.8.3)
project(dwa_mobs_planner)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            pcl_conversions
            roscpp
            tf
            message_generation
        )
find_package(Boost REQUIRED
    COMPONENTS
        thread
        )
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    OdometryMovingObstacles.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs
   std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/DWAMobsPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        dwa_mobs_planner
        mobstacle_cost_function
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        pluginlib
        roscpp
        message_runtime
        costmap_2d
)

add_library(dwa_mobs_planner src/dwa_mobs_planner.cpp src/dwa_mobs_planner_ros.cpp)
add_library(mobstacle_cost_function src/mobstacle_cost_function)

target_link_libraries(dwa_mobs_planner
    base_local_planner
    mobstacle_cost_function
    ${catkin_LIBRARIES}
    )
target_link_libraries(mobstacle_cost_function
    base_local_planner
    ${catkin_LIBRARIES}
    )
add_dependencies(dwa_mobs_planner dwa_mobs_planner_gencfg)
add_dependencies(mobstacle_cost_function base_local_planner)
add_dependencies(dwa_mobs_planner ${catkin_EXPORTED_TARGETS})

install(TARGETS dwa_mobs_planner
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


