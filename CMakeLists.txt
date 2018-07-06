cmake_minimum_required(VERSION 3.5.1)
project(ros_gammon_rs485)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_runtime message_generation)

add_service_files(FILES GammonExchange.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES ros_gammon_rs485
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        #  DEPENDS system_lib
)


include_directories("${PROJECT_SOURCE_DIR}/include/")
include_directories("${ROJECT_SOURCE_DIR}/src/")
include_directories(${catkin_INCLUDE_DIRS})

set(serial_SRCS src/serial.cc include/serial/serial.h include/serial/v8stdint.h)
list(APPEND serial_SRCS src/impl/unix.cc)
add_library(Serial ${serial_SRCS})


add_executable(ros_gammon_rs485 src/main.cpp)
add_executable(ros_gammon_node src/ros_gammon_node.cpp)
target_link_libraries(ros_gammon_node ${catkin_LIBRARIES} Serial)
target_link_libraries(ros_gammon_rs485 Serial)

