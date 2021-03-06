project(simplePMP)

MESSAGE(STATUS "PROJECT NAME: " ${PROJECT_NAME})
MESSAGE(STATUS "CURRENT DIRECTORY: " ${CMAKE_CURRENT_SOURCE_DIR}/include)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

find_package(YARP REQUIRED)
#find_package(BOOST REQUIRED)
find_package(ICUB REQUIRED)
#find_package(Gazebo REQUIRED)

set(YARP_CLOCK "/clock")
set(GYPLUGINS_INCLUDE_DIRS ${/home/yeshi/gyplugins/include/gazebo})
set(GYPLUINGS_LIBRARIES ${/home/yeshi/gyplugins/lib})


set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
MESSAGE(STATUS "SOURCE DIRECTORY: " ${SOURCE_DIR})
set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
MESSAGE(STATUS "HEADER DIRECTORY: " ${HEADER_DIR})

# This is for the Clock plugin of YARP GAZEBO - Find package can't locateGazeboYarpPlugins as of now
include_directories("${HEADER_DIR}/autogenerated/include/")
set(CLOCK_THRIFT_SOURCES "${HEADER_DIR}/autogenerated/src/thrift/ClockServer.cpp")
set(CLOCK_THRIFT_HEADERS "${HEADER_DIR}/autogenerated/include/thrift/ClockServer.h")

# Search for source code.
FILE(GLOB SOURCE_FILES ${SOURCE_DIR}/*.cpp ${SOURCE_DIR}/*.cc ${SOURCE_DIR}/*.c)
FILE(GLOB HEADER_FILES ${HEADER_DIR}/*.h ${HEADER_DIR}/*.hpp)

INCLUDE_DIRECTORIES(${HEADER_DIR} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${GYPLUGINS_INCLUDE_DIRS} ${CLOCK_THRIFT_HEADERS})

	

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES} ${CLOCK_THRIFT_SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${YARP_LIBRARIES} iKin ${GYPLUINGS_LIBRARIES} ${ICUB_LIBRARIES})

