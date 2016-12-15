cmake_minimum_required(VERSION 2.8.11)
project(ros_node_example)

## Set @result to list of all @curdir subdirectories which contain <include> folder and set them to @result
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child}/include)
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Set trikRuntime project directory
set(TRIK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../)
set(PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

message(CUR_DIR-${CMAKE_CURRENT_SOURCE_DIR})
## Find all trikRuntime subdirectories which contain <include> folder
SUBDIRLIST(TRIK_SUBDIRS ${TRIK_DIR})

FOREACH(subdir ${TRIK_SUBDIRS})
  LIST(APPEND TRIK_INCLUDE_DIRS ${TRIK_DIR}/${subdir}/include)
ENDFOREACH()

## Set Qt directory
SET(QT_DIR /opt/trik-sdk/sysroots/arm926ejste-oe-linux-gnueabi/usr/include/qt4/)

## Find all Qt subdirectories
FILE(GLOB children RELATIVE ${QT_DIR} ${QT_DIR}/*)
SET(QT_INCLUDE_DIRS "")
FOREACH(child ${children})
  LIST(APPEND QT_INCLUDE_DIRS ${QT_DIR}/${child})
ENDFOREACH()

message(CUSTOM_HEADERS-${PROJECT_DIR})
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${TRIK_INCLUDE_DIRS}
        ${QT_DIR}
	${QT_INCLUDE_DIRS}
        ${PROJECT_DIR}/headers
	
)

link_directories(${TRIK_DIR}/bin/arm-release/)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} trikControl QtCore QtGui)
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
