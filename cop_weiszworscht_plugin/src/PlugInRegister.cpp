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




/*Includes for Descriptor Plugins*/
  /*interface of cognitive_perception*/
#include "Descriptor.h"

#include "WWDetector.h"

/*Includes for LocaeAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "LocateAlgorithm.h"

#include "cpp/HalconCpp.h"
//This procedure simply hands the exception object to the C++ exception handling via throw:
void MyWWHalconExceptionHandler(const Halcon::HException& except)
{
  throw except;
}


using namespace cop;

void loadLibWW(void) __attribute__ ((constructor));

void loadLibWW(void)
{
    Halcon::HException::InstallHHandler(&MyWWHalconExceptionHandler);
}

#define PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(class_name, class_type, base_class_type) \
  POCO_BEGIN_NAMED_MANIFEST(class_name, base_class_type) \
  POCO_EXPORT_CLASS(class_type) \
  POCO_END_MANIFEST

/*Sensor Plugins*/
/*Reading Plugin*/

/*Descriptor Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(WeisswurstContainer, WeisswurstContainer, Descriptor);

/*LocateAlgorithm Plugins*/
PLUGINLIB_REGISTER_NO_NAMESPACE_CLASS(WWDetector, WWDetector, LocateAlgorithm);
