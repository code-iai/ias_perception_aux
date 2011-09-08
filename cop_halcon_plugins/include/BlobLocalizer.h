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


#ifndef BLOBLOCALIZER_H
#define BLOBLOCALIZER_H
#include "LocateAlgorithm.h"
#include "Image.h"
#include "Camera.h"

#define XML_NODE_BLOBLOCALIZER "BlobLocalizer"
namespace cop
{
  class BlobLocalizer :
    public LocateAlgorithm
  {

  public:
    BlobLocalizer();
    BlobLocalizer(XMLTag* tag);
    ~BlobLocalizer();

    virtual std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);
    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual bool TrackingPossible(const Reading& , const Signature& , RelPose* ){return false;}
    virtual XMLTag* Save();

    std::vector<RelPose*> Inner(Image* img, RelPose* camPose,Calibration* calib,RelPose* lastKnownPose, Signature& object, int &numOfObjects, double& qualityMeasure);

    virtual std::string GetName(){return XML_NODE_BLOBLOCALIZER;}

  };
}
#endif /*BLOBLOCALIZER_H*/
