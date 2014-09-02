PROJECT(inspect_orf)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
set(CMAKE_BUILD_TYPE Debug)

## Find packages
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

## Add structure from motion
ADD_SUBDIRECTORY(SfM)
INCLUDE_DIRECTORIES(SfM)
LINK_LIBRARIES(SfMLib)

## Add src and target
ADD_EXECUTABLE(Main utils.cpp camera.cpp orf.cpp halo.cpp main.cpp triangulator.cpp orientation.cpp calibration.cpp SfM/Visualization.cpp)

## Add MESA SR libaries
FIND_PATH( MESASR_INCLUDE_PATH libMesaSR.h
	/usr/include
	/usr/local/include
	/sw/include
	/opt/local/include
	DOC "The directory where libMesaSR.h resides")
FIND_LIBRARY( MESASR_LIBRARY
	NAMES mesasr
	PATHS
	/usr/lib64
	/usr/lib
	/usr/local/lib64
	/usr/local/lib
	/sw/lib
	/opt/local/lib
	DOC "The MESASR library")
TARGET_LINK_LIBRARIES(Main ${MESASR_LIBRARY})
INCLUDE_DIRECTORIES(${MESASR_INCLUDE_PATH})

## Add ueye
FIND_PATH(UEYEUSB_INCLUDE_DIR  ueye.h ueye_tools.h
	${CMAKE_INCLUDE_PATH}
	$ENV{HOME}/include/
	/usr/include/opencv
	$ENV{ProgramFiles}/IDS/ueye/Develop/include
)
FIND_LIBRARY(UEYEUSB_LIBRARY
	NAMES
	libueye_api.so
	PATHS
	${CMAKE_LIBRARY_PATH}
	/usr/lib
	$ENV{ProgramFiles}/IDS/ueye/Develop/lib
)
TARGET_LINK_LIBRARIES(Main ${UEYEUSB_LIBRARY})
INCLUDE_DIRECTORIES(${UEYEUSB_INCLUDE_DIR})

## Add OpenCV
TARGET_LINK_LIBRARIES(Main ${OpenCV_LIBS})

## Add PCL
if(EIGEN_INCLUDE_DIRS) # if compiling with PCL, it will bring Eigen with it
	message(STATUS "will use Eigen")
	include_directories(${EIGEN_INCLUDE_DIRS})
	add_definitions( -DUSE_EIGEN )
endif()
LINK_LIBRARIES(Main ${PCL_LIBRARY_DIRS})
target_link_libraries(Main ${PCL_LIBRARIES})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

## Add Boost library
TARGET_LINK_LIBRARIES(Main ${BOOST_LIBRARIES})

