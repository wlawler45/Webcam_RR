    
cmake_minimum_required(VERSION 3.3)
project(Webcam_RR)

set (CMAKE_CXX_STANDARD 11)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )



find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteur_standard_robdef_cpp REQUIRED)
find_package( OpenCV REQUIRED )


include_directories(${RobotRaconteur_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS 
	
	com.robotraconteur.geometry.robdef
	com.robotraconteur.geometryf.robdef
	com.robotraconteur.geometryi.robdef
	com.robotraconteur.identifier.robdef
	com.robotraconteur.resource.robdef
	com.robotraconteur.imaging.camerainfo.robdef
	com.robotraconteur.image.robdef
	com.robotraconteur.uuid.robdef
	com.robotraconteur.datetime.robdef
	com.robotraconteur.device.robdef
	com.robotraconteur.param.robdef
	com.robotraconteur.imaging.robdef
	com.robotraconteur.sensordata.robdef
	com.robotraconteur.imu.robdef
	com.robotraconteur.units.robdef
	com.robotraconteur.datatype.robdef
	com.robotraconteur.sensor.robdef
	
	MASTER_HEADER
	
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
	IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../../robotraconteur_standard_robdef_cpp/robdef/group1
	
	
	
	)
if (MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()



add_executable(Webcam_RR
	${RR_THUNK_HDRS}
	${RR_THUNK_SRCS}
	main.cpp
	Webcam_RR.cpp
	Webcam_RR.h)


target_link_libraries( Webcam_RR ${OpenCV_LIBS} )
target_link_libraries(Webcam_RR RobotRaconteurCore ${Boost_LIBRARIES} ${RobotRaconteurCore_Extra_libraries} )