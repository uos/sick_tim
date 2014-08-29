cmake_minimum_required(VERSION 2.8.3)
project(sick_tim)

find_package(catkin REQUIRED
    COMPONENTS
      sensor_msgs
      diagnostic_updater
      dynamic_reconfigure
      driver_base)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

generate_dynamic_reconfigure_options(cfg/SickTim.cfg)

catkin_package(
    DEPENDS libusb
    CATKIN_DEPENDS sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${libusb_INCLUDE_DIRS})

add_library(sick_tim_3xx
    src/sick_tim_common.cpp
    src/sick_tim_common_usb.cpp
    src/abstract_parser.cpp)
add_dependencies(sick_tim_3xx ${PROJECT_NAME}_gencfg)
target_link_libraries(sick_tim_3xx
    ${catkin_LIBRARIES}
    ${libusb_LIBRARIES})

add_executable(sick_tim310s01
    src/sick_tim310s01.cpp
    src/sick_tim310s01_parser.cpp)
target_link_libraries(sick_tim310s01 sick_tim_3xx)

add_executable(sick_tim310
    src/sick_tim310.cpp
    src/sick_tim310_parser.cpp)
target_link_libraries(sick_tim310 sick_tim_3xx)

add_executable(sick_tim310_1130000m01
    src/sick_tim310_1130000m01.cpp
    src/sick_tim310_1130000m01_parser.cpp)
target_link_libraries(sick_tim310_1130000m01 sick_tim_3xx)

add_executable(sick_tim551_2050001
    src/sick_tim551_2050001.cpp
    src/sick_tim_common_tcp.cpp
    src/sick_tim551_2050001_parser.cpp)
target_link_libraries(sick_tim551_2050001 sick_tim_3xx)

add_executable(sick_tim_datagram_test
    test/sick_tim_datagram_test.cpp
    src/sick_tim310s01_parser.cpp
    src/abstract_parser.cpp)
target_link_libraries(sick_tim_datagram_test sick_tim_3xx)

install(TARGETS sick_tim_3xx
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    TARGETS
        sick_tim310s01
        sick_tim310
        sick_tim310_1130000m01
        sick_tim551_2050001
        sick_tim_datagram_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
