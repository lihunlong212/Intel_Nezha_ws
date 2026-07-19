cmake_minimum_required(VERSION 3.8)
project(apriltag_tuner_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio aruco)

add_executable(apriltag_tuner_node src/apriltag_tuner_node.cpp)
ament_target_dependencies(apriltag_tuner_node rclcpp)
target_include_directories(apriltag_tuner_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltag_tuner_node ${OpenCV_LIBS})

install(TARGETS
  apriltag_tuner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
