cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(sick_tim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${libusb_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(sick_tim_3xx
    src/sick_tim_common.cpp
    src/sick_tim_common_usb.cpp
    src/sick_tim_common_mockup.cpp
    src/abstract_parser.cpp)
ament_target_dependencies(sick_tim_3xx diagnostic_updater example_interfaces rclcpp sensor_msgs)
target_link_libraries(sick_tim_3xx ${libusb_LIBRARIES} ${Boost_LIBRARIES})

add_executable(sick_tim551_2050001
    src/sick_tim551_2050001.cpp
    src/sick_tim_common_tcp.cpp
    src/sick_tim551_2050001_parser.cpp)
ament_target_dependencies(sick_tim551_2050001 diagnostic_updater example_interfaces rclcpp sensor_msgs)
target_link_libraries(sick_tim551_2050001 sick_tim_3xx)

install(TARGETS sick_tim_3xx
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  sick_tim551_2050001
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES include/${PROJECT_NAME}/abstract_parser.h
              include/${PROJECT_NAME}/sick_tim_common.h
              include/${PROJECT_NAME}/sick_tim_common_usb.h
    DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch/
    DESTINATION share/launch)

install(DIRECTORY meshes/
    DESTINATION share/meshes)

install(DIRECTORY urdf/
    DESTINATION share/urdf)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
    
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

