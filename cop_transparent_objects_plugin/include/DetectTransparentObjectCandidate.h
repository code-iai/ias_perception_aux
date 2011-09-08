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


#ifndef DETECTTRANSPARENTOBJECTCANDIDATE_H
#define DETECTTRANSPARENTOBJECTCANDIDATE_H

#include "LocateAlgorithm.h"
#include "cpp/HalconCpp.h"


void Segmentation_Glasses (Halcon::Hobject IntensityImage, Halcon::Hobject DistanceImage,  Halcon::HTuple DomainRow,
	Halcon::HTuple DomainCol, Halcon::Hobject *CandidateRegion);

void Segmentation_Glasses_Split(Halcon::Hobject CandidateRegion, std::vector<std::pair<Halcon::HTuple, Halcon::HTuple> > &Cluster_RC, Halcon::HTuple CamParam);

void splitDim(Halcon::Hobject Image, Halcon::Hobject Region, const Halcon::HTuple &Mean,
		double min_object_size_for_split, Halcon::HTuple &split_x, Halcon::HTuple &size_x);

#define XML_NODE_DETECTTRANSPARENTOBJECTCANDIDATE "DetectTransparentObjectCandidate"
namespace cop
{



  class DetectTransparentObjectCandidate :  public LocateAlgorithm
  {
  public:
    DetectTransparentObjectCandidate();
    ~DetectTransparentObjectCandidate(void);

    XMLTag* Save();
    // Public attribute accessor methods
    //
    std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure);

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors);

    virtual std::string GetName(){return XML_NODE_DETECTTRANSPARENTOBJECTCANDIDATE;}

    virtual void SetData(XMLTag* tag);
  private:

  };
}
#endif /*DETECTTRANSPARENTOBJECTCANDIDATE_H*/
