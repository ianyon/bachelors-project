cmake_minimum_required(VERSION 2.8.3)
project(bachelors_final_project)

## Supress warnings if using CMake 3 or higher
if(POLICY CMP0054)
    cmake_policy(SET CMP0054 OLD)
endif()
if(POLICY CMP0045)
    cmake_policy(SET CMP0045 OLD)
endif()
if(POLICY CMP0058)
    cmake_policy(SET CMP0058 OLD)
endif()
##############################################

# Cotire Precompiled headers
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "~/catkin_ws/src/bachelors_final_project/cmake/Modules")#"${CMAKE_SOURCE_DIR}/cmake/Modules/")
include(cotire)

if(EXISTS "/home/yoko/catkin_ws/devel")
    set(CATKIN_DEVEL_PREFIX "/home/yoko/catkin_ws/devel")
    set(CMAKE_INSTALL_PREFIX "/home/yoko/catkin_ws/install")
else()
    set(CATKIN_DEVEL_PREFIX "/home/ian/catkin_ws/devel")
    set(CMAKE_INSTALL_PREFIX "/home/ian/catkin_ws/install")
endif()
add_definitions(-Wno-deprecated)

# ccache doesnt support --gsplit-dwarf (DEBUG FISSION)
#find_program(CCACHE_FOUND ccache)
#if(CCACHE_FOUND)
#    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "ccache clang -stdlib=libstdc++")
#    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "ccache")
#endif(CCACHE_FOUND)

## Compiler options
#set (CMAKE_CXX_COMPILER "/usr/lib/ccache/clang++")
#set (CMAKE_C_COMPILER "/usr/lib/ccache/clang")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++") #-Wall -O0

#set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")
#set (CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS}")

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  dynamic_reconfigure
  moveit_ros_planning_interface
)

## Add include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Generate dynamic_reconfigure
generate_dynamic_reconfigure_options(
        cfg/Parameters.cfg
)

## Declare the catkin package
catkin_package(
        CATKIN_DEPENDS
            pcl_conversions
            pcl_ros
            roscpp
            sensor_msgs
            dynamic_reconfigure
)
message(STATUS "${CMAKE_SOURCE_DIR}")
FILE (GLOB_RECURSE all_headers ${PROJECT_SOURCE_DIR}/bachelors_final_project/include/*.h)
FILE (GLOB_RECURSE all_headers ${CMAKE_SOURCE_DIR}/include/*.h)
FILE (GLOB_RECURSE all_cfgs ${CMAKE_SOURCE_DIR}/cfg/*.cfg)
FILE (GLOB_RECURSE all_cfgs ${PROJECT_SOURCE_DIR}/bachelors_final_project/cfg/*.cfg)
set(sources src/example.cpp
        src/cloud_segmentator.cpp
        src/grasp_point_detector.cpp

        src/grasp_sampler.cpp
        src/ellipse_operations.cpp
        src/grasp_filter.cpp
        src/grasp_filter_utils.cpp
        src/grasp_ranker.cpp
        src/detection_utils.cpp

        src/visualization/visualization_thread.cpp
        src/visualization/base_visualizer.cpp
        src/visualization/segmentation_visualizer.cpp
        src/visualization/detection_visualizer.cpp

        src/utils.cpp
        src/bounding_box.cpp)

add_executable(example ${sources} ${all_headers} ${all_cfgs})

add_executable(test_data src/test_publisher.cpp)
add_executable(test_ransac src/ransac.cpp)
add_executable(test_ransac_normals src/ransac_normals.cpp)

#Ensures that configuration files are generated before main target
add_dependencies(example ${PROJECT_NAME}_gencfg)

target_link_libraries(test_data
        ${catkin_LIBRARIES}
        libvtkCommon.so
        libvtkFiltering.so )
target_link_libraries(ransac
        ${catkin_LIBRARIES}
        libvtkCommon.so
        libvtkFiltering.so )
target_link_libraries(ransac_normals
        ${catkin_LIBRARIES}
        libvtkCommon.so
        libvtkFiltering.so )

target_link_libraries(example
        ${catkin_LIBRARIES}
        libvtkCommon.so
        libvtkFiltering.so )
cotire(example)
add_dependencies(example_unity ${PROJECT_NAME}_gencfg)

#execute_process(COMMAND rosservice call /gazebo/pause_physics)
#add_custom_target(stop_physics
#    COMMAND rosservice call /gazebo/pause_physics)
#add_dependencies(example stop_physics)
#add_dependencies(example_unity stop_physics)

#get_target_property(_unitySource example COTIRE_CXX_UNITY_SOURCE)
#get_target_property(_prefixHeader example COTIRE_CXX_PREFIX_HEADER)
#get_target_property(_precompiledHeader example COTIRE_CXX_PRECOMPILED_HEADER)
#get_target_property(_unityTargetName example COTIRE_UNITY_TARGET_NAME)

#message(STATUS "${_unitySource}")
#message(STATUS "${_prefixHeader}")
#message(STATUS "${_precompiledHeader}")
#message(STATUS "${_unityTargetName}")

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
