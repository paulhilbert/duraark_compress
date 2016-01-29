cmake_minimum_required(VERSION 2.8.8)
project(duraark_compress)

set (duraark_compress_VERSION_MAJOR 0)
set (duraark_compress_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
	add_definitions(-fPIC)
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++1y)
    add_definitions(-Wall)
    add_definitions(-Wno-unused-function)
    add_definitions(-Wno-unused-local-typedefs)
    add_definitions(-Wno-deprecated-declarations)
endif()

find_package(OpenCV REQUIRED core highgui imgproc)
find_package(PCL COMPONENTS common io search octree)
find_package(PrimitiveDetection)
find_package(PCLCompress)
find_package(E57PCL)

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
message(STATUS ${obj})
if (OPENCV_CORE_FOUND AND OPENCV_HIGHGUI_FOUND AND PCL_FOUND AND PRIMITIVE_DETECTION_FOUND AND PCLCOMPRESS_FOUND AND E57PCL_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${PRIMITIVE_DETECTION_INCLUDE_DIRS})
	include_directories(${PCLCOMPRESS_INCLUDE_DIRS})
	include_directories(${E57PCL_INCLUDE_DIRS})

    find_package(Boost COMPONENTS system filesystem program_options regex)
    add_executable(duraark_compress ${obj} "apps/duraark_compress.cpp")
    target_link_libraries(duraark_compress ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PRIMITIVE_DETECTION_LIBRARIES} ${PCLCOMPRESS_LIBRARIES} ${E57PCL_LIBRARIES} "dl")
    add_executable(duraark_decompress ${obj} "apps/duraark_decompress.cpp")
    target_link_libraries(duraark_decompress ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PRIMITIVE_DETECTION_LIBRARIES} ${PCLCOMPRESS_LIBRARIES} ${E57PCL_LIBRARIES} "dl")

    # install binary
    install (TARGETS duraark_compress DESTINATION bin)
    # install binary
    install (TARGETS duraark_decompress DESTINATION bin)
    # install header
#   install (DIRECTORY include/ DESTINATION include/duraark_compress)
endif()
