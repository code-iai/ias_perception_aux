cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
#INCLUDE_DIRECTORIES($ENV{HALCONROOT}/include)
#LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})
#LINK_LIBRARIES(halconcpp)

find_package(PkgConfig)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )

#HALCON include + libs
INCLUDE_DIRECTORIES($ENV{HALCONROOT}/include)
LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

LINK_LIBRARIES(m halcon halconcpp python2.6)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rosbuild_add_library(bvw
src/PVWObjectClassification.cpp
)
