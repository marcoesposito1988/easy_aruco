cmake_minimum_required(VERSION 2.8.12)
project(easy_aruco)

find_package(catkin REQUIRED COMPONENTS sensor_msgs cv_bridge tf2_ros tf2_geometry_msgs)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
        include/ArucoMarkerDetector.h
        include/CharucoBoardDetector.h
        include/Detector.h
        include/Utils.h
        src/ArucoMarkerDetector.cpp
        src/CharucoBoardDetector.cpp
        src/Detector.cpp
        src/Utils.cpp
        src/easy_aruco_node.cpp
        )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

catkin_install_python(PROGRAMS
        scripts/create_charuco_board.py
        scripts/create_aruco_marker.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
