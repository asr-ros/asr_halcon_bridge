cmake_minimum_required(VERSION 2.8.3)
project(asr_halcon_bridge)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Halcon REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp sensor_msgs
	LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
        DEPENDS Halcon
)

include_directories(
	include
	${Halcon_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}/halcon_image.cpp
    src/${PROJECT_NAME}/halcon_pointcloud.cpp
)

target_link_libraries(${PROJECT_NAME}
	${CATKIN_LIBRARIES}
	${Halcon_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
