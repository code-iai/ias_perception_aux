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


/*Includes for Reading Plugins*/
  /*interface of cognitive_perception*/
#include "Reading.h"

/* Image Includes also for ReadingConverter Plugins*/



/*Includes for Descriptor Plugins*/
  /*interface of cognitive_perception*/
#include "Descriptor.h"

#include "ShapeModel.h"
#include "ColorModelFromCAD.h"
#include "SurfaceDetection.h"

/*Includes for LocaeAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "LocateAlgorithm.h"

#include "ShapeBased3D.h"
#include "SurfaceDetection.h"

/*Includes for RefineAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "RefineAlgorithm.h"

#include "RFAPointDescrByShape.h"
#include "RFAColorByShape.h"
#include "RFASurfaceByShape.h"
#include "RFASurfaceByCov.h"

#include "cpp/HalconCpp.h"
//This procedure simply hands the exception object to the C++ exception handling via throw:
void MyCADHalconExceptionHandler(const Halcon::HException& except)
{
  throw except;
}


using namespace cop;

void loadLibCad(void) __attribute__ ((constructor));

void loadLibCad(void)
{
    Halcon::HException::InstallHHandler(&MyCADHalconExceptionHandler);
    Halcon::set_system("parallelize_operators", "true");
    Halcon::set_system("reentrant", "true");
    Halcon::set_system("thread_pool", "true");
    Halcon::set_system("thread_num", 8);
    Halcon::set_system("tsp_thread_num", 8);
    
    
}

#define PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(class_name, class_type, base_class_type) \
  POCO_BEGIN_NAMED_MANIFEST(class_name, base_class_type) \
  POCO_EXPORT_CLASS(class_type) \
  POCO_END_MANIFEST

/*Sensor Plugins*/
/*Reading Plugin*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(ImageFile, Image, Reading);

/*Descriptor Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(ShapeModel, ShapeModel, Descriptor);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SurfaceModel, SurfaceModel, Descriptor);

/*LocateAlgorithm Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(ShapeBased3DAlg, ShapeBased3D, LocateAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SurfaceDetection, SurfaceDetection, LocateAlgorithm);


/*RefineAlgorithm Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(RFAColorByShape, RFAColorByShape, RefineAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(RFASurfaceByShape, RFASurfaceByShape, RefineAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(RFASurfaceByCov, RFASurfaceByCov, RefineAlgorithm);
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(SurfaceModelExtraction, SurfaceModelExtraction, RefineAlgorithm);

