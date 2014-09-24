PROJECT(inspect_orf)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
set(CMAKE_BUILD_TYPE Debug)

set(COMPILE_EXAMPLES)

## Find packages
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

## Add src and target
ADD_LIBRARY(inspect_sensors utils.cpp calibration.cpp camera.cpp orf.cpp halo.cpp triangulator.cpp haloTriangulator.cpp orientation.cpp visualization.cpp)

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
TARGET_LINK_LIBRARIES(inspect_sensors ${MESASR_LIBRARY})
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
TARGET_LINK_LIBRARIES(inspect_sensors ${UEYEUSB_LIBRARY})
INCLUDE_DIRECTORIES(${UEYEUSB_INCLUDE_DIR})

## Add OpenCV
TARGET_LINK_LIBRARIES(inspect_sensors ${OpenCV_LIBS})

## Add PCL
if(EIGEN_INCLUDE_DIRS) # if compiling with PCL, it will bring Eigen with it
	message(STATUS "will use Eigen")
	include_directories(${EIGEN_INCLUDE_DIRS})
	add_definitions( -DUSE_EIGEN )
endif()
LINK_LIBRARIES(inspect_sensors ${PCL_LIBRARY_DIRS})
target_link_libraries(inspect_sensors ${PCL_LIBRARIES})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

## Add Boost library
TARGET_LINK_LIBRARIES(inspect_sensors ${BOOST_LIBRARIES})

# Build Main program
add_executable(Main main.cpp)
target_link_libraries(Main inspect_sensors)

# # Build examples
# add_executable(orfCapture examples/orfCapture.cpp)
# target_link_libraries(orfCapture inspect_sensors)
# add_executable(orfSave examples/orfSave.cpp)
# target_link_libraries(orfSave inspect_sensors)
# add_executable(orf3DCloud examples/orf3DCloud.cpp)
# target_link_libraries(orf3DCloud inspect_sensors)
# add_executable(orfFile3DCloud examples/orfFile3DCloud.cpp)
# target_link_libraries(orfFile3DCloud inspect_sensors)
# add_executable(orfMeasDepth examples/orfMeasDepth.cpp)
# target_link_libraries(orfMeasDepth inspect_sensors)
# add_executable(orfMeasXYZ examples/orfMeasXYZ.cpp)
# target_link_libraries(orfMeasXYZ inspect_sensors)
# add_executable(orfCalib examples/orfCalib.cpp)
# target_link_libraries(orfCalib inspect_sensors)
# 
# add_executable(stereoCapture examples/stereoCapture.cpp)
# target_link_libraries(stereoCapture inspect_sensors)
# add_executable(stereoSave examples/stereoSave.cpp)
# target_link_libraries(stereoSave inspect_sensors)
# add_executable(stereoCalib examples/stereoCalib.cpp)
# target_link_libraries(stereoCalib inspect_sensors)
# add_executable(stereoProject examples/stereoProject.cpp)
# target_link_libraries(stereoProject inspect_sensors)
# add_executable(stereoProjectLinear examples/stereoProjectLinear.cpp)
# target_link_libraries(stereoProjectLinear inspect_sensors)
# add_executable(stereoProjectIterative examples/stereoProjectIterative.cpp)
# target_link_libraries(stereoProjectIterative inspect_sensors)
# add_executable(stereoProjectHaloPoint examples/stereoProjectHaloPoint.cpp)
# target_link_libraries(stereoProjectHaloPoint inspect_sensors)
# 
# add_executable(haloSave examples/haloSave.cpp)
# target_link_libraries(haloSave inspect_sensors)
# add_executable(haloCapture examples/haloCapture.cpp)
# target_link_libraries(haloCapture inspect_sensors)
# add_executable(haloCalib examples/haloCalib.cpp)
# target_link_libraries(haloCalib inspect_sensors)
# add_executable(haloFileCalib examples/haloFileCalib.cpp)
# target_link_libraries(haloFileCalib inspect_sensors)

