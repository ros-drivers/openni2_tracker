cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
					nodelet
					image_transport
					tf)
# Find OpenNI2
find_package(PkgConfig)
pkg_check_modules(OpenNI2 REQUIRED libopenni2)

# Find Nite2
message(STATUS "NITE2_INCLUDE: " $ENV{NITE2_INCLUDE})
message(STATUS "NITE2_REDIST64: " $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS $ENV{NITE2_INCLUDE})
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS $ENV{NITE2_REDIST64}
	     PATH_SUFFIXES lib) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp
)

include_directories(include
  ${catkin_INCLUDEDIR}
  ${OpenNI2_INCLUDE_DIRS}
  ${Nite2_INCLUDEDIR})
link_directories(${OpenNI2_LIBRARY_DIRS})
add_library(openni2_tracker src/openni2_tracker_nodelet.cpp)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES openni2_tracker_nodelet.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
