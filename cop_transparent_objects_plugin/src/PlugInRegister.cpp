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

#include "TransparentObjectCandidate.h"
#include "TransparentObject.h"

/*Includes for LocaeAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "LocateAlgorithm.h"

#include "DetectTransparentObjectCandidate.h"

/*Includes for RefineAlgorithm Plugins*/
  /*interface of cognitive_perception*/
#include "RefineAlgorithm.h"

#include "DetectTransparentObject.h"

void loadLibXY(void) __attribute__ ((constructor));

void loadLibXY(void)
{
 printf("LoadLibXY done!\n");
}
using namespace cop;

/*Descriptor Plugins*/
PLUGINLIB_REGISTER_CLASS(TransparentObjectCandidate, TransparentObjectCandidate, Descriptor);
PLUGINLIB_REGISTER_CLASS(TransparentObject,TransparentObject, Descriptor);

/*LocateAlgorithm Plugins*/
PLUGINLIB_REGISTER_CLASS(DetectTransparentObjectCandidate, DetectTransparentObjectCandidate, LocateAlgorithm);

/*RefineAlgorithm Plugins*/
PLUGINLIB_REGISTER_CLASS(DetectTransparentObject, DetectTransparentObject, RefineAlgorithm);
