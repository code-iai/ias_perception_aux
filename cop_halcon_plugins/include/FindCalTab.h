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


#ifndef FINDCALTAB_H
#define FINDCALTAB_H

#include "LocateAlgorithm.h"
#include "Camera.h"
#define XML_NODE_FINDCALTAB "FindCalTab"

namespace cop
{
  class CalTab;

  class FindCalTab :
    public LocateAlgorithm
  {
  public:
    FindCalTab();
    FindCalTab(XMLTag*);
    ~FindCalTab(void);

    XMLTag* Save();

    // Public attributes
    //


    // Public attribute accessor methods
    //
    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    std::vector<RelPose*> Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure);
    std::vector<RelPose*> Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, CalTab* object, int &numOfObjects, double& qualityMeasure);

    virtual std::string GetName(){return XML_NODE_FINDCALTAB;}

  };
}
#endif /*FINDCALTAB_H*/
