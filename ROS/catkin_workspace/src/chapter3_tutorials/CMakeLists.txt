cmake_minimum_required(VERSION 2.8.3)
project(chapter3_tutorials)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
#      dynamic_reconfigure
#      driver_base
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

find_package(OpenCV)
find_package(PCL REQUIRED)

# add_service_files(FILES SetSpeed.srv)

# generate_messages(DEPENDENCIES std_msgs)

# generate_dynamic_reconfigure_options(cfg/DynamicParam.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      std_msgs
      geometry_msgs
      sensor_msgs
      visualization_msgs
#      dynamic_reconfigure
#      driver_base
      diagnostic_updater
      cv_bridge
      image_transport
      pcl_conversions
      interactive_markers)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_executable(example8 src/example8.cpp)
target_link_libraries(example8 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS
    example8
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
