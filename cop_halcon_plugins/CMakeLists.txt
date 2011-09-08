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

ROSBUILD_ADD_LIBRARY(VisionPlugins
./src/PlugInRegister.cpp
./src/BlobLocalizer.cpp
./src/RegionOI.cpp
#./src/DxfReader.cpp
./src/Blob.cpp
#./src/ShapeBased3D.cpp
#./src/ShapeModel.cpp
./src/CameraDriver.cpp
./src/ColorBased.cpp
#./src/ColorModel.cpp
./src/SimulatedCamera.cpp
./src/DescriptorBased.cpp
./src/TemplateBased.cpp
./src/TexturedModel.cpp
./src/Image.cpp
./src/Camera.cpp
#./src/PRAShapeVsPointDescr.cpp	
#./src/RFAColorByShape.cpp
#./src/RFAPointDescrByShape.cpp 	
./src/DeformShapeModel.cpp
./src/RFADeformByCluster.cpp
./src/PointDescrModel.cpp
./src/DeformShapeBased.cpp
./src/SearchParams3d.cpp
./src/CalTab.cpp
./src/FindCalTab.cpp
./src/ROIGenerator.cpp
./src/HClusterDetector.cpp
./src/ROSCamera.cpp
./src/SupportingPlaneDetector.cpp
./src/SupportingPlaneDescriptor.cpp
./src/SimulatedLocate.cpp
./src/CheckColorClass.cpp
#./src/dxfwriter.cpp
./src/TwoInOneAlg.cpp
./src/IntersectTwoRuns.cpp
./src/File.cpp
#./src/ClassifyObject.cpp
#./src/RFAClassByDPs.cpp
./src/StereoSensor.cpp
./src/RelPoseHTuple.cpp
./src/PanCakeDetector.cpp
./src/DetectPlate.cpp
#./src/SurfaceDetection.cpp
)

set(DEBUG TRUE CACHE BOOL TRUE)
if (DEBUG )
  ADD_DEFINITIONS(-g -DDEFORMSHAPE_AVAILABLE  -D_DEBUG  -DBOOST_THREAD -DLOGFILE  -DHALCONIMG)
else (DEBUG)
  ADD_DEFINITIONS(-o3 -DDEFORMSHAPE_AVAILABLE -DBOOST_THREAD -DLOGFILE   -DHALCONIMG)
endif (DEBUG)
