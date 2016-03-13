cmake_minimum_required(VERSION 2.8.3)
project(mtrespider)
 
## Find catkin macros and libraries
find_package(catkin 
 COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system chrono timer)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
 #  Message2.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mtrespider
  CATKIN_DEPENDS image_transport roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
add_library(pid-controller SHARED IMPORTED
    /usr/local/lib/libpid-controller.so
)
set_target_properties(pid-controller PROPERTIES
    IMPORTED_LOCATION /usr/local/lib/libpid-controller.so
)

add_library (wiringPi SHARED IMPORTED
    /usr/local/lib/libwiringPi.so
)
set_target_properties(wiringPi PROPERTIES
    LINKER_LANGUAGE cpp
    IMPORTED_LOCATION /usr/local/lib/libwiringPi.so
)


add_library(iir
    include/mtrespider/iir/Bessel.cpp
    include/mtrespider/iir/Biquad.cpp
    include/mtrespider/iir/Butterworth.cpp
    include/mtrespider/iir/Cascade.cpp
    include/mtrespider/iir/ChebyshevI.cpp
    include/mtrespider/iir/ChebyshevII.cpp
    include/mtrespider/iir/Custom.cpp
    include/mtrespider/iir/Elliptic.cpp
    include/mtrespider/iir/Legendre.cpp
    include/mtrespider/iir/PoleFilter.cpp
    include/mtrespider/iir/RBJ.cpp
    include/mtrespider/iir/RootFinder.cpp
    include/mtrespider/iir/State.cpp
)

## Declare a cpp executable
add_executable(ferrosensor src/ferrosensor.cpp)
add_executable(transducerarm src/transducerarm.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(ferrosensor mtrespider_generate_messages_cpp)
add_dependencies(transducerarm mtrespider_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(ferrosensor
    ${catkin_LIBRARIES}
    wiringPi
)
target_link_libraries(transducerarm
    ${catkin_LIBRARIES}
    pid-controller
    wiringPi
)
