cmake_minimum_required(VERSION 3.1.0)
project(codrha_local_planner)

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
            nlopt
            message_generation
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

# Adds libnlopt directory
link_directories(
    ${catkin_LIBRARY_DIRS}
    # ${CMAKE_BINARY_DIR}/codrha_local_planner/lib/nlopt/bin
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/CODRHAPlanner.cfg
)

add_message_files(
    FILES
    trajectory_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES codrha_local_planner
  CATKIN_DEPENDS
  dynamic_reconfigure
  pluginlib
  roscpp
  nlopt
  message_runtime
)

# gcc debugging configuration
# set(CMAKE_BUILD_TYPE Debug)
# set( GCC_COVERAGE_COMPILE_FLAGS "-Og -g" )

# gcc production configuration
set( GCC_COVERAGE_COMPILE_FLAGS "-Ofast" )

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
# set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

add_library(codrha_local_planner
  src/codrha_planner.cpp
  src/codrha_planner_ros.cpp
  src/codrha_controller.cpp
  src/codrha_comm_ros.cpp)

target_compile_features(codrha_local_planner PRIVATE cxx_range_for)
target_compile_features(codrha_local_planner PRIVATE cxx_delegating_constructors)

# Add libnlopt as a library
target_link_libraries(codrha_local_planner ${catkin_LIBRARIES})
# target_link_libraries(codrha_local_planner ${catkin_LIBRARIES})

add_dependencies(codrha_local_planner codrha_local_planner_gencfg)
# add_dependencies(codrha_local_planner nav_msgs_gencpp)

install(TARGETS codrha_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
