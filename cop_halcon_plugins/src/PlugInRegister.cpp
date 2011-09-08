 /*
  * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "pluginlib/class_list_macros.h"


/*Includes for Sensor Plugins*/
  /*interface of cognitive_perception*/
#include "Sensor.h"

#include "CameraDriver.h"
#include "SimulatedCamera.h"
#include "ROSCamera.h"
#include "StereoSensor.h"

/*Includes for Reading Plugins*/
  /*interface of cognitive_perception*/
#include "Reading.h"

#include "Image.h"
/* Image Includes also for ReadingConverter Plugins*/



/*Includes for Descriptor Plugins*/
  /*interface of cognitive_perception*/
#include "Descriptor.h"

#include "CalTab.h"
#include "DeformShapeModel.h"
#include "ColorClass.h"
#include "Blob.h"
#include "SupportingPlaneDescriptor.h"
#include "CircleDescriptor.h"

/*Includes for LocaeAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "LocateAlgorithm.h"

#include "FindCalTab.h"
#include "DeformShapeBased.h"
#include "CheckColorClass.h"
#include "BlobLocalizer.h"
#include "TwoInOneAlg.h"
#include "SupportingPlaneDetector.h"
#include "SimulatedLocate.h"
#include "IntersectTwoRuns.h"
#include "HClusterDetector.h"
#include "PanCakeDetector.h"
#include "DetectPlate.h"

/*Includes for RefineAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "RefineAlgorithm.h"
#include "RFADeformByCluster.h"
#include "RFAClassByDPs.h"

/*#include "RFAPointDescrByShape.h"*/
/*#include "RFAColorByShape.h"*/


#include "cpp/HalconCpp.h"
//This procedure simply hands the exception object to the C++ exception handling via throw:
void MyHalconExceptionHandler(const Halcon::HException& except)
{
  throw except;
}


using namespace cop;

void loadLib(void) __attribute__ ((constructor));

void loadLib(void)
{
    printf("Register Image Conversions\n");
    Image::RegisterImageConverter();
    printf("Register HALCON Exception Handler\n");
    Halcon::HException::InstallHHandler(&MyHalconExceptionHandler);
}

#define PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(class_name, class_type, base_class_type) \
  POCO_BEGIN_NAMED_MANIFEST(class_name, base_class_type) \
  POCO_EXPORT_CLASS(class_type) \
  POCO_END_MANIFEST

/*Sensor Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(CameraDriver, CameraDriver, Sensor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(CameraDriverRelay, CameraDriverRelay, Sensor);

PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SimulatedCamera, SimulatedCamera, Sensor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(ROSCOPCamera, ROSCOPCamera, Sensor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(StereoSensor, StereoSensor, Sensor);

/*Reading Plugin*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(ImageFile, Image, Reading);

/*Descriptor Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(CalTab, CalTab, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(DeformShapeModel, DeformShapeModel, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(PanCakeMaker, PanCakeMaker, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(ColorClass, ColorClass, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(Blob, Blob, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SupportingPlaneDescriptor, SupportingPlaneDescriptor, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(CircleDescriptor, CircleDescriptor, Descriptor);

/*LocateAlgorithm Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(FindCalTab, FindCalTab, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(DeformShapeBased, DeformShapeBased, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(DeformShapeBasedAlg, DeformShapeBased, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(CheckColorClass, CheckColorClass, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(BlobLocalizer, BlobLocalizer, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(TwoInOneAlg, TwoInOneAlg, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SupportingPlaneDetector, SupportingPlaneDetector, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SimulatedLocate, SimulatedLocate, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(IntersectTwoRuns, IntersectTwoRuns, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(HClusterDetector, HClusterDetector, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(PanCakeDetector, PanCakeDetector, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(DetectPlate, DetectPlate, LocateAlgorithm);


/*RefineAlgorithm Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(RFADeformByCluster, RFADeformByCluster, RefineAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(RFAClassByDPs, RFAClassByDPs, RefineAlgorithm);

//PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(RFAColorByShape, RFAColorByShape, RefineAlgorithm);

