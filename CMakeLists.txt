cmake_minimum_required(VERSION 2.8.3)
project(openni2_tracker)
find_package(catkin REQUIRED COMPONENTS 
					roscpp
					roslib
					tf)
find_package(orocos_kdl REQUIRED)

catkin_package()

include(ExternalProject)
## Check architecture 32 or 64 bit for precompiled OpenNI2 and NiTE2 Libraries
if (CMAKE_SIZEOF_VOID_P MATCHES "8")
	ExternalProject_Add(
		OpenNI2
		PREFIX ${CMAKE_CURRENT_BINARY_DIR}
		URL http://www.openni.ru/wp-content/uploads/2013/01/OpenNI-Linux-x64-2.1.0.tar.zip
		URL_MD5 be1f12b964fe07f4a3243b946c6ce387
		UPDATE_COMMAND tar -xf <SOURCE_DIR>/OpenNI-Linux-x64-2.1.0.tar.bz2
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
	)
	ExternalProject_Add(
		NiTE2
		PREFIX ${CMAKE_CURRENT_BINARY_DIR}
		URL http://www.openni.ru/wp-content/uploads/NiTE-Linux-x64-2.0.0.tar.zip
		URL_MD5 4244e25bebc87a15fc761b7371107494
		UPDATE_COMMAND tar -xf <SOURCE_DIR>/NiTE-Linux-x64-2.0.0.tar.bz2
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
	)

	add_custom_target(OpenNI2_copy)
	add_dependencies(OpenNI2_copy OpenNI2)
	add_custom_command(TARGET OpenNI2_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/OpenNI2/OpenNI-2.1.0-x64 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x64)

	add_custom_target(NiTE2_copy)
	add_dependencies(NiTE2_copy NiTE2)
	add_custom_command(TARGET NiTE2_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/NiTE2/NiTE-2.0.0 		${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/NiTE-2.0.0)

	add_custom_target(OpenNI2_conf_copy)
	add_dependencies(OpenNI2_conf_copy OpenNI2)
	add_custom_command(TARGET OpenNI2_conf_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/OpenNI2/OpenNI-2.1.0-x64/Redist/OpenNI2	$ENV{HOME}/.ros/OpenNI2)

	add_custom_target(NiTE2_conf_copy)
	add_dependencies(NiTE2_conf_copy NiTE2)
	add_custom_command(TARGET NiTE2_conf_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/NiTE2/NiTE-2.0.0/Redist/NiTE2	$ENV{HOME}/.ros/NiTE2)

	set(OpenNI2_INCLUDEDIR 	${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x64/Include)
	set(OpenNI2_LIBRARIES 	${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x64/Redist/libOpenNI2.so
	 						${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x64/Redist/OpenNI2/Drivers/libPS1080.so
	 						${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x64/Redist/OpenNI2/Drivers/libOniFile.so)
	set(Nite2_INCLUDEDIR 	${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/NiTE-2.0.0/Include)
	set(Nite2_LIBRARY 		${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/NiTE-2.0.0/Redist/libNiTE2.so)
elseif (CMAKE_SIZEOF_VOID_P MATCHES "4")
	ExternalProject_Add(
		OpenNI2
		PREFIX ${CMAKE_CURRENT_BINARY_DIR}
		URL http://www.openni.ru/wp-content/uploads/2013/01/OpenNI-Linux-x86-2.1.0.tar.zip
		URL_MD5 827b37db761b5885177d3e414cec7762
		UPDATE_COMMAND tar -xf <SOURCE_DIR>/OpenNI-Linux-x86-2.1.0.tar.bz2
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
	)
	ExternalProject_Add(
		NiTE2
		PREFIX ${CMAKE_CURRENT_BINARY_DIR}
		URL http://www.openni.ru/wp-content/uploads/NiTE-Linux-x86-2.0.0.tar.zip
		URL_MD5 30d66a07447a73e5bcd8f28af6628606
		UPDATE_COMMAND tar -xf <SOURCE_DIR>/NiTE-Linux-x86-2.0.0.tar.bz2
		CONFIGURE_COMMAND ""
		BUILD_COMMAND ""
	)

	add_custom_target(OpenNI2_copy)
	add_dependencies(OpenNI2_copy OpenNI2)
	add_custom_command(TARGET OpenNI2_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/OpenNI2/OpenNI-2.1.0-x86 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x86)

	add_custom_target(NiTE2_copy)
	add_dependencies(NiTE2_copy NiTE2)
	add_custom_command(TARGET NiTE2_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/NiTE2/NiTE-2.0.0 		${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/NiTE-2.0.0)

	add_custom_target(OpenNI2_conf_copy)
	add_dependencies(OpenNI2_conf_copy OpenNI2)
	add_custom_command(TARGET OpenNI2_conf_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/OpenNI2/OpenNI-2.1.0-x86/Redist/OpenNI2	$ENV{HOME}/.ros/OpenNI2)

	add_custom_target(NiTE2_conf_copy)
	add_dependencies(NiTE2_conf_copy NiTE2)
	add_custom_command(TARGET NiTE2_conf_copy PRE_BUILD
						COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_BINARY_DIR}/src/NiTE2/NiTE-2.0.0/Redist/NiTE2	$ENV{HOME}/.ros/NiTE2)

	set(OpenNI2_INCLUDEDIR 	${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x86/Include)
	set(OpenNI2_LIBRARIES 	${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x86/Redist/libOpenNI2.so
	 						${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x86/Redist/OpenNI2/Drivers/libPS1080.so
	 						${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/OpenNI-2.1.0-x86/Redist/OpenNI2/Drivers/libOniFile.so)
	set(Nite2_INCLUDEDIR 	${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/NiTE-2.0.0/Include)
	set(Nite2_LIBRARY 		${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/NiTE-2.0.0/Redist/libNiTE2.so)
endif (CMAKE_SIZEOF_VOID_P MATCHES "8")

include_directories(${catkin_INCLUDEDIR} ${OpenNI2_INCLUDEDIR} ${Nite2_INCLUDEDIR})

add_executable(openni2_tracker src/openni2_tracker.cpp)
add_dependencies(openni2_tracker OpenNI2_copy NiTE2_copy OpenNI2_conf_copy NiTE2_conf_copy)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

install(TARGETS openni2_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
