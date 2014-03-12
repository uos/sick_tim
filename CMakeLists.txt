cmake_minimum_required(VERSION 2.8.3)
project(sick_tim3xx)

find_package(catkin REQUIRED
    COMPONENTS
      sensor_msgs
      diagnostic_updater
      dynamic_reconfigure
      driver_base)

find_path(libusb_INCLUDE_DIRS 
          NAMES libusb.h
          HINTS /usr/include/libusb-1.0)
find_library(libusb_LIBRARIES
             NAMES usb-1.0
             HINTS /usr/lib/ /usr/x86_64-linux-gnu/
             PATH_SUFFIXES lib)

generate_dynamic_reconfigure_options(cfg/SickTim3xx.cfg)

catkin_package(
    DEPENDS libusb
    CATKIN_DEPENDS sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${libusb_INCLUDE_DIRS})

add_library(sick
    src/sick_tim3xx_common.cpp
    src/sick_tim3xx_common_usb.cpp
    src/abstract_parser.cpp)
add_dependencies(sick ${PROJECT_NAME}_gencfg)
target_link_libraries(sick
    ${catkin_LIBRARIES}
    ${libusb_LIBRARIES})

add_executable(sick_tim310s01
    src/sick_tim310s01.cpp
    src/sick_tim310s01_parser.cpp)
target_link_libraries(sick_tim310s01 sick)

add_executable(sick_tim310
    src/sick_tim310.cpp
    src/sick_tim310_parser.cpp)
target_link_libraries(sick_tim310 sick)

add_executable(sick_tim310_1130000m01
    src/sick_tim310_1130000m01.cpp
    src/sick_tim310_1130000m01_parser.cpp)
target_link_libraries(sick_tim310_1130000m01 sick)

add_executable(sick_tim551_2050001
    src/sick_tim551_2050001.cpp
    src/sick_tim3xx_common_tcp.cpp
    src/sick_tim551_2050001_parser.cpp)
target_link_libraries(sick_tim551_2050001 sick)

add_executable(sick_tim3xx_datagram_test
    test/sick_tim3xx_datagram_test.cpp
    src/sick_tim310s01_parser.cpp
    src/abstract_parser.cpp)
target_link_libraries(sick_tim3xx_datagram_test sick)

install(TARGETS sick
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    TARGETS
        sick_tim310s01
        sick_tim310
        sick_tim310_1130000m01
        sick_tim551_2050001
        sick_tim3xx_datagram_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
