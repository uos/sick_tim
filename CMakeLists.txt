cmake_minimum_required(VERSION 2.8.3)
project(sick_tim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(C_CXX_FLAGS "-Wall -Werror -Wformat=2 -Wsign-compare -Wmissing-field-initializers -Wwrite-strings -ggdb -fvisibility=hidden -fno-common")
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(C_CXX_FLAGS "${C_CXX_FLAGS} -Wnewline-eof")
  endif()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_CXX_FLAGS} -Wmissing-prototypes -Wold-style-definition -Wstrict-prototypes")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${C_CXX_FLAGS} -Wmissing-declarations")
  # Clang's integerated assembler does not support debug symbols.
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -Wa,-g")
  endif()
elseif(MSVC)
  set(MSVC_DISABLED_WARNINGS_LIST
      "C4061" # enumerator 'identifier' in switch of enum 'enumeration' is not
              # explicitly handled by a case label
              # Disable this because it flags even when there is a default.
      "C4100" # 'exarg' : unreferenced formal parameter
      "C4127" # conditional expression is constant
      "C4200" # nonstandard extension used : zero-sized array in
              # struct/union.
      "C4204" # nonstandard extension used: non-constant aggregate initializer
      "C4221" # nonstandard extension used : 'identifier' : cannot be
              # initialized using address of automatic variable
      "C4242" # 'function' : conversion from 'int' to 'uint8_t',
              # possible loss of data
      "C4244" # 'function' : conversion from 'int' to 'uint8_t',
              # possible loss of data
      "C4245" # 'initializing' : conversion from 'long' to
              # 'unsigned long', signed/unsigned mismatch
      "C4267" # conversion from 'size_t' to 'int', possible loss of data
      "C4371" # layout of class may have changed from a previous version of the
              # compiler due to better packing of member '...'
      "C4388" # signed/unsigned mismatch
      "C4296" # '>=' : expression is always true
      "C4350" # behavior change: 'std::_Wrap_alloc...'
      "C4365" # '=' : conversion from 'size_t' to 'int',
              # signed/unsigned mismatch
      "C4389" # '!=' : signed/unsigned mismatch
      "C4464" # relative include path contains '..'
      "C4510" # 'argument' : default constructor could not be generated
      "C4512" # 'argument' : assignment operator could not be generated
      "C4514" # 'function': unreferenced inline function has been removed
      "C4548" # expression before comma has no effect; expected expression with
              # side-effect" caused by FD_* macros.
      "C4610" # struct 'argument' can never be instantiated - user defined
              # constructor required.
      "C4623" # default constructor was implicitly defined as deleted
      "C4625" # copy constructor could not be generated because a base class
              # copy constructor is inaccessible or deleted
      "C4626" # assignment operator could not be generated because a base class
              # assignment operator is inaccessible or deleted
      "C4668" # 'symbol' is not defined as a preprocessor macro, replacing with
              # '0' for 'directives'
              # Disable this because GTest uses it everywhere.
      "C4706" # assignment within conditional expression
      "C4710" # 'function': function not inlined
      "C4711" # function 'function' selected for inline expansion
      "C4800" # 'int' : forcing value to bool 'true' or 'false'
              # (performance warning)
      "C4820" # 'bytes' bytes padding added after construct 'member_name'
      "C5026" # move constructor was implicitly defined as deleted
      "C5027" # move assignment operator was implicitly defined as deleted
      "C2220"
      "C4619"
      "C5031"
      "C4643"
      "C4265"
      "C3861"
	  "C2509"
	  "C2143"
      )
  set(MSVC_LEVEL4_WARNINGS_LIST
      # See https://connect.microsoft.com/VisualStudio/feedback/details/1217660/warning-c4265-when-using-functional-header
      "C4265" # class has virtual functions, but destructor is not virtual
      )
  string(REPLACE "C" " -wd" MSVC_DISABLED_WARNINGS_STR
                            ${MSVC_DISABLED_WARNINGS_LIST})
  string(REPLACE "C" " -w4" MSVC_LEVEL4_WARNINGS_STR
                            ${MSVC_LEVEL4_WARNINGS_LIST})
  set(CMAKE_C_FLAGS   "-Wall -WX ${MSVC_DISABLED_WARNINGS_STR} ${MSVC_LEVEL4_WARNINGS_STR}")
  set(CMAKE_CXX_FLAGS "-Wall -WX ${MSVC_DISABLED_WARNINGS_STR} ${MSVC_LEVEL4_WARNINGS_STR}")
  set(CMAKE_ASM_NASM_FLAGS "-g cv8")
  add_definitions(-D_HAS_EXCEPTIONS=0)
  add_definitions(-DWIN32_LEAN_AND_MEAN)
  add_definitions(-DNOMINMAX)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS) # Allow use of fopen
endif()

find_package(catkin REQUIRED
    COMPONENTS
      diagnostic_updater
      dynamic_reconfigure
      roscpp
      roslaunch
      sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(libusb libusb-1.0)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(cfg/SickTim.cfg)

catkin_package(
    DEPENDS Boost libusb
    CATKIN_DEPENDS roscpp sensor_msgs diagnostic_updater dynamic_reconfigure
    LIBRARIES sick_tim_3xx
    INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${libusb_INCLUDE_DIRS})


add_executable(sick_tim551_2050001
    src/sick_tim551_2050001.cpp
    src/sick_tim_common_tcp.cpp
    src/sick_tim551_2050001_parser.cpp)
target_link_libraries(sick_tim551_2050001 sick_tim_3xx)

add_executable(sick_mrs1000
  src/sick_mrs1000.cpp
  src/sick_mrs1000_communication.cpp
  src/sick_tim_common_tcp.cpp
  src/scan_and_cloud_parser.cpp
  src/sick_mrs1000_parser.cpp)
target_link_libraries(sick_mrs1000 sick_tim_3xx)

add_executable(sick_tim_datagram_test
    test/sick_tim_datagram_test.cpp
    src/sick_tim310s01_parser.cpp
    src/abstract_parser.cpp)
target_link_libraries(sick_tim_datagram_test sick_tim_3xx)

install(
    TARGETS
        sick_tim551_2050001
        sick_mrs1000
        sick_tim_datagram_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES include/${PROJECT_NAME}/abstract_parser.h
              include/${PROJECT_NAME}/sick_tim_common.h
              include/${PROJECT_NAME}/sick_tim_common_usb.h
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif(CATKIN_ENABLE_TESTING)
