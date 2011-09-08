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


#ifndef SUPPORTINGPLANEDETECTOR_H
#define SUPPORTINGPLANEDETECTOR_H

#include "LocateAlgorithm.h"
#include "Camera.h"

#define XML_NODE_SUPPORTINGPLANEDETECTOR "SupportingPlaneDetector"

namespace cop
{
  class SupportingPlaneDetector :
      public LocateAlgorithm
  {
  public:
      SupportingPlaneDetector(void);
      virtual ~SupportingPlaneDetector(void);

      virtual std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

      std::vector<RelPose*> Inner(Image* img, RelPose* campose, Calibration* calib, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure, int index);


      virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

      virtual bool TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose);
      virtual XMLTag* Save();

  };
}
#endif /*SUPPORTINGPLANEDETECTOR_H*/
