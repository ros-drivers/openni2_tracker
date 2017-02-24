cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					roscpp
					roslib
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

catkin_package()

include_directories(${catkin_INCLUDE_DIRS}
		    ${OpenNI2_INCLUDE_DIRS}
		    ${Nite2_INCLUDEDIR})
link_directories(${OpenNI2_LIBRARY_DIRS})
add_executable(openni2_tracker src/openni2_tracker.cpp)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

install(TARGETS openni2_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
