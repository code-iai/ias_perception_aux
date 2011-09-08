PROJECT(cop)

cmake_minimum_required(VERSION 2.6)

INCLUDE("${CMAKE_MODULE_PATH}/MakeDocu.cmake" OPTIONAL)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


find_package(PkgConfig)

pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
include_directories(${LIBXML2_INCLUDE_DIR})
link_directories( ${LIBXML2_LIBRARY_DIRS} )
add_definitions( ${LIBXML2_CFLAGS} )

#HALCON include + libs
INCLUDE_DIRECTORIES($ENV{HALCONROOT}/include)
LINK_DIRECTORIES($ENV{HALCONROOT}/lib/$ENV{HALCONARCH})

LINK_LIBRARIES(m halcon halconcpp)

ROSBUILD_ADD_LIBRARY(ToolPlugins
./src/FindLines.cpp 
./src/FindPlanes.cpp
./src/find_lines.cpp
)

set(DEBUG TRUE CACHE BOOL TRUE)
if (DEBUG )
  ADD_DEFINITIONS(-g  -D_DEBUG  -DBOOST_THREAD -DLOGFILE)
else (DEBUG)
  ADD_DEFINITIONS(-g  -DBOOST_THREAD -DLOGFILE )
endif (DEBUG)
