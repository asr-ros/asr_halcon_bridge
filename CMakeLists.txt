cmake_minimum_required(VERSION 3.8)
project(asr_halcon_bridge)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Halcon REQUIRED)
if(NOT ${Halcon_FOUND})
   message(WARNING  "Skip processing ${PROJECT_NAME}, because HALCON library is missing!!! (see http://wiki.ros.org/asr_halcon_bridge)")
   return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
	include
	${Halcon_INCLUDE_DIRS}
)

set(HEADER_FILES 
    include/${PROJECT_NAME}/halcon_image.hpp
    include/${PROJECT_NAME}/halcon_pointcloud.hpp)

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/halcon_image.cpp
    src/${PROJECT_NAME}/halcon_pointcloud.cpp
    ${HEADER_FILES}
)

target_include_directories(asr_halcon_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_14)

ament_target_dependencies(${PROJECT_NAME} 
  Halcon sensor_msgs)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
