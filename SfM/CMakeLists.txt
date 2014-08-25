project(SfM)
cmake_minimum_required(VERSION 2.8)

find_package(PCL REQUIRED)
if(EIGEN_INCLUDE_DIRS) # if compiling with PCL, it will bring Eigen with it
	message(STATUS "will use Eigen")
	include_directories(${EIGEN_INCLUDE_DIRS})
	add_definitions( -DUSE_EIGEN )
endif()

set(SSBA_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/SfM/3rdparty/SSBA-3.0/build" CACHE PATH "Directory to find SSBA libs")

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
link_directories(
	${SSBA_LIBRARY_DIR}
	)

IF(APPLE)
	set( COCOA_LIBS ${CMAKE_OSX_SYSROOT}/System/Library/Frameworks/Cocoa.framework )
	set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -F/usr/local/lib -L/opt/local/lib")
ENDIF(APPLE)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${SSBA_LIBRARY_DIR})
include_directories(${SSBA_LIBRARY_DIR}/../)

set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D__SFM__DEBUG__" )

# Core functions of SfM
file(GLOB SFM_LIB_SOURCES	
	Distance.* 
	Triangulation.* 
	FindCameraMatrices.*
	MultiCameraDistance.*
	IDistance.*
	MultiCameraPnP.*
	Common.*
	IFeatureMatcher.*
	RichFeatureMatcher.*
	OFFeatureMatcher.*
	BundleAdjuster.*
	GPUSURFFeatureMatcher.*
	AbstractFeatureMatcher.*
	SfMUpdateListener.*
)

add_library( SfMLib ${SFM_LIB_SOURCES} )
if(MSVC)
	set_target_properties(SfMLib PROPERTIES COMPILE_FLAGS "/openmp")
	target_link_libraries(SfMLib V3D COLAMD )
endif()
if(APPLE)
	# default is to build SSBA as static libs...
	target_link_libraries(SfMLib 
		"${SSBA_LIBRARY_DIR}/libV3D.a" 
		"${SSBA_LIBRARY_DIR}/libCOLAMD.a"
		)
		
	set_target_properties(SfMLib PROPERTIES COMPILE_FLAGS "-fopenmp")
	set_target_properties(SfMLib PROPERTIES LINK_FLAGS "-fopenmp")
endif()


# UI part
add_executable(SfM 
	Visualization.cpp
	Visualization.h
	main.cpp
	)


target_link_libraries(SfM 
			SfMLib
			${OpenCV_LIBS}
			${PCL_LIBRARIES}
			V3D
			colamd
			)

