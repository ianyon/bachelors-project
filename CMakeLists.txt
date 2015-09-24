cmake_minimum_required(VERSION 2.8.3)
project(bachelors_final_project)

## Supress warnings if using CMake 3 or higher
if(POLICY CMP0054)
    cmake_policy(SET CMP0054 OLD)
endif()
if(POLICY CMP0045)
    cmake_policy(SET CMP0045 OLD)
endif()
##############################################

set(CATKIN_DEVEL_PREFIX "/home/ian/catkin_ws/devel")
set(CMAKE_INSTALL_PREFIX "/home/ian/catkin_ws/install")

add_definitions(-Wno-deprecated)

#find_program(CCACHE_FOUND ccache)
#if(NOT CCACHE_FOUND)
#    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "/usr/bin/ccache clang -Qunused-arguments -fcolor-diagnostics")
#    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "/usr/bin/ccache clang -Qunused-arguments -fcolor-diagnostics")
#endif(NOT CCACHE_FOUND)
## To speed up PCL compilation
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -v -std=gnu++98 -O0") #-Wall -O0 -v
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -v")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -v")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -v")
set (CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} -v")
#set (CMAKE_CXX_COMPILER "${DCMAKE_STATIC_LINKER_FLAGS} -v")
#/usr/bin/g++-4.9


## Find system dependencies
#find_package(Boost REQUIRED COMPONENTS thread)
#find_package(Eigen REQUIRED)
#find_package(PCL REQUIRED)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  dynamic_reconfigure
)

## Add include directories
include_directories(
        #include
        #${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        #${Eigen_INCLUDE_DIRS}
        #${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
        #${Boost_LIBRARY_DIRS}
        #${Eigen_LIBRARY_DIRS}
        #${PCL_LIBRARY_DIRS}
)

## Generate dynamic_reconfigure
generate_dynamic_reconfigure_options(
        cfg/Parameters.cfg
)

## Declare the catkin package
catkin_package(
        #INCLUDE_DIRS
            #include
        #LIBRARIES
            #pcl_ros_filters
        CATKIN_DEPENDS
            pcl_conversions
            pcl_ros
            roscpp
            sensor_msgs
            dynamic_reconfigure
        #DEPENDS
            #Boost
            #Eigen
            #PCL
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#  /usr/local/bin/clang++
#-D CMAKE_CXX_COMPILER=/usr/bin/ccache clang -Qunused-arguments -fcolor-diagnostics
#if(Boost_FOUND)
#    message(STATUS "Boost found Success!")
#endif()
message(STATUS ${CMAKE_CXX_COMPILER})
message("cmake source dir" ${CMAKE_SOURCE_DIR})

FILE (GLOB_RECURSE clion_all_headers ${CMAKE_SOURCE_DIR}/include/*.h)
ADD_CUSTOM_TARGET(all_clion
        SOURCES ${clion_all_headers}
)

add_executable(example src/example.cpp src/data_handler.cpp src/data_visualizer.cpp src/utils.cpp)
add_dependencies(example ${PROJECT_NAME}_gencfg)
target_link_libraries(example
        #${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        #${PCL_LIBRARIES}
        libvtkCommon.so
        libvtkFiltering.so )

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
